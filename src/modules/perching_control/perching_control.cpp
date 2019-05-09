
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_module.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>
#include <commander/px4_custom_mode.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <matrix/matrix/math.hpp>
#include <float.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>
#include <controllib/blocks.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/optical_flow_tau_theta.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_constraints.h>
#include "Utility/ControlMath.hpp"

#include <lib/FlightTasks/FlightTasks.hpp>
//#include <modules/mc_pos_control/PositionControl.hpp>

extern "C" __EXPORT int perching_control_main(int argc, char *argv[]);

class MulticopterPerchingControl : public ModuleBase<MulticopterPerchingControl>, public control::SuperBlock, public ModuleParams
{
public:
    MulticopterPerchingControl();
    ~MulticopterPerchingControl();
    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static MulticopterPerchingControl *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:
    matrix::Vector3f _thr_sp{};

    struct optical_flow_tau_theta_s ofd;

    struct optical_flow_s of;

    struct distance_sensor_s dis;

    struct vehicle_attitude_s att;

    struct vehicle_attitude_setpoint_s _att_sp;

    struct vehicle_control_mode_s _control_mode;

    struct vehicle_status_s _vehicle_status;

    vehicle_constraints_s _constraints{}; /**< variable constraints */

    orb_advert_t    _att_sp_pub{nullptr};           /**< attitude setpoint publication */

    orb_id_t _attitude_setpoint_id{nullptr};

    orb_advert_t _vehicle_control_mode_pub;

    int optical_front_sub;

    int optical_downward_sub; 

    int dis_z_sub;

    int att_sub;

    int _params_sub;

    int vehicle_control_mode_sub;

    int vehicle_status_sub;

    control::BlockDerivative _h_deriv;
    control::BlockDerivative _vel_z_deriv;
    control::BlockDerivative _xmotion_deriv;
    FlightTasks _flight_tasks;
    //PositionControl _positioncontrol;


    float output_h;//filtered h
    float previous_h; //for fi
    float sf = 0.7; //filter smooh factor
    float _thr_int_z = 0.0f;
    float _thr_int_y = 0.0f;
    float roll;
    float pitch;
    float yaw;
    float yaw_sp;
    float yaw_rate_sp;
    float flow_divergence;
    float direction_guidence;
    float xmotion, ymotion;
    float H;
    float vel_z_sp;
    float vel_z;
    float acc_z;


    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::PC_Z_H>) h_sp,
        (ParamFloat<px4::params::PC_Z_KP>) k_p,
        (ParamFloat<px4::params::PC_Z_VEL_KP>) k_vel_p,
        (ParamFloat<px4::params::PC_Z_VEL_KI>) k_vel_i,
        (ParamFloat<px4::params::PC_Z_VEL_KD>) k_vel_d,
        (ParamFloat<px4::params::PC_OFD_KP>) k_ofd_p,
        (ParamFloat<px4::params::PC_OFD_GOAL>) desired_ofd,
        (ParamFloat<px4::params::PC_NP_KP>) k_m_p,
        (ParamFloat<px4::params::PC_NP_KI>) k_m_i,
        (ParamFloat<px4::params::PC_NP_KD>) k_m_d,
        (ParamFloat<px4::params::PC_YAW_KP>) k_yaw,
        (ParamFloat<px4::params::PC_YAW_RATE_KP>) k_yaw_rate,
        (ParamFloat<px4::params::MPC_THR_HOVER>) MPC_THR_HOVER,
        (ParamFloat<px4::params::MPC_THR_MIN>) MPC_THR_MIN,
        (ParamFloat<px4::params::MPC_THR_MAX>) MPC_THR_MAX,
        (ParamFloat<px4::params::MPC_MANTHR_MIN>) MPC_MANTHR_MIN,
        (ParamFloat<px4::params::MPC_XY_VEL_MAX>) MPC_XY_VEL_MAX,
        (ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) MPC_Z_VEL_MAX_DN,
        (ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) MPC_Z_VEL_MAX_UP,
        (ParamFloat<px4::params::MPC_TILTMAX_AIR>) MPC_TILTMAX_AIR_rad, // maximum tilt for any position controlled mode in radians
        (ParamFloat<px4::params::MPC_MAN_TILT_MAX>) MPC_MAN_TILT_MAX_rad  // maximum til for stabilized/altitude mode in radians
    );
    int parameters_update(bool force);
    void poll_subscriptions();
    void updateConstraints(const vehicle_constraints_s &constraints);

};

int MulticopterPerchingControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
The perching controller use PID on z direction and non perching direction to maintain the altitude and the trajectory. 
Z is estimated by distance sensor, non perching direction movement is estimated by downward facing optical flow.
A constant optical flow divergence control policy is used for perching direction control. OFD is from front facing optical flow array.
The multicopter will alway fly towards it's own heading, and yaw angle is directed by the front facing optical flow array.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("perching_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

MulticopterPerchingControl::MulticopterPerchingControl() :
    SuperBlock(nullptr, "PC"),
    ModuleParams(nullptr),
    _h_deriv(this, "HD"),
    _vel_z_deriv(this, "VELZ"),
    _xmotion_deriv(this, "MD")
    //_positioncontrol(this)
{
    parameters_update(true);
}

MulticopterPerchingControl::~MulticopterPerchingControl(){

}

int MulticopterPerchingControl::parameters_update(bool force)
{
    bool updated;
    struct parameter_update_s param_upd;

    orb_check(_params_sub, &updated);

    if(updated) {
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
    }

    if(updated || force){
        ModuleParams::updateParams();
        SuperBlock::updateParams();

        _flight_tasks.handleParameterUpdate();  
    }
    
    MPC_TILTMAX_AIR_rad.set(math::radians(MPC_TILTMAX_AIR_rad.get()));
    MPC_MAN_TILT_MAX_rad.set(math::radians(MPC_MAN_TILT_MAX_rad.get()));
    return OK;
}

int
MulticopterPerchingControl::print_status()
{
    //if (_flight_tasks.isAnyTaskActive()) {
        //PX4_INFO("Running, active flight task: %i", _flight_tasks.getActiveTask());
    //} else {
        PX4_INFO("Running");
    //}
    return 0;
}

void MulticopterPerchingControl::poll_subscriptions()
{
    //orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

    /*message subscriber*/
        bool updated;
        orb_check(optical_front_sub, &updated);
        if(updated){
            
            orb_copy(ORB_ID(optical_flow_tau_theta), optical_front_sub, &ofd);
            flow_divergence = ofd.tau;
            direction_guidence = ofd.theta;
        }

        orb_check(att_sub, &updated);
        if(updated){
            
            orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
            yaw = Eulerf(Quatf(att.q)).psi();
            roll = Eulerf(Quatf(att.q)).phi();
            pitch = Eulerf(Quatf(att.q)).theta();
        }


       orb_check(optical_downward_sub, &updated);
       if(updated){
            
            orb_copy(ORB_ID(optical_flow), optical_downward_sub, &of);
            ymotion = of.pixel_flow_x_integral;
            xmotion = of.pixel_flow_y_integral;//quick fix, swap the x,y here bacause I used xmotion for the controller
       }
      
       orb_check(dis_z_sub, &updated);
       if(updated){ 
             orb_copy(ORB_ID(distance_sensor), dis_z_sub, &dis);
             float h = dis.current_distance * cos(roll) * cos(pitch);
             if(h>=200){h = 2.0;} else if(h<=0){h=0.0;}
             output_h = sf * h + (1-sf)* previous_h;
             previous_h = output_h;
       }

       orb_check(vehicle_control_mode_sub, &updated);
       if(updated){
        orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_sub, &_control_mode);
       }

       orb_check(vehicle_status_sub, &updated);

       if (updated) {
            orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &_vehicle_status);

            // set correct uORB ID, depending on if vehicle is VTOL or not
            if (!_attitude_setpoint_id) {
                if (_vehicle_status.is_vtol) {
                    _attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);

                } else {
                    _attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
                }
            }
        }
}

void MulticopterPerchingControl::run()
{
    px4_usleep(50000);
    hrt_abstime time_stamp_last_loop = hrt_absolute_time();

    /* subscribe to customised optical_flow_tayu_theta topic */
    optical_front_sub = orb_subscribe(ORB_ID(optical_flow_tau_theta));

    optical_downward_sub = orb_subscribe(ORB_ID(optical_flow));

    dis_z_sub = orb_subscribe(ORB_ID(distance_sensor));

    att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

    vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

    _vehicle_control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &_control_mode);

    //orb_set_interval(att_sub, 20); // 50 Hz updates

    parameters_update(true);
    poll_subscriptions();
    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[1];
    fds[0].fd = att_sub;
    fds[0].events = POLLIN;

    while(!should_exit()){
        //PX4_INFO("perching controller started");
        // poll for new data on the attitude topic (wait for up to 20ms)
        int poll_ret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 20);
        //if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
        //    PX4_ERR("Got no data within a second");
        //}else 
        if (poll_ret < 0) {
            PX4_ERR("poll error %d, %d", poll_ret, errno);
            px4_usleep(50000);
            continue;
        }else{
            if(fds[0].revents & POLLIN){
                orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
                yaw = Eulerf(Quatf(att.q)).psi();
                roll = Eulerf(Quatf(att.q)).phi();
                pitch = Eulerf(Quatf(att.q)).theta();
            }
        }

        poll_subscriptions();
        parameters_update(true);

        //PX4_INFO("SENSOR_DATA:\t%8.4f\t%8.4f\t%8.4f", (double)output_h,(double)xmotion,(double)ymotion);
        
        //set _dt
        const hrt_abstime time_stamp_current = hrt_absolute_time();
        setDt((time_stamp_current - time_stamp_last_loop) / 1e6f);
        time_stamp_last_loop = time_stamp_current;

        vehicle_constraints_s constraints = _flight_tasks.getConstraints();
        updateConstraints(constraints);

    /*if (orb_copy(ORB_ID(vehicle_attitude), att_sub, &att) == PX4_OK && PX4_ISFINITE(att.q[0])) {
               yaw = Eulerf(Quatf(att.q)).psi();
               roll = Eulerf(Quatf(att.q)).phi();
               pitch = Eulerf(Quatf(att.q)).theta();
            }*/
    /*thrust in D direction*/
    while((_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_PERCH)&&(_control_mode.flag_control_perch_enabled)){

            poll_subscriptions();
            parameters_update(true);

            bool topic_changed = false;

            if (_control_mode.flag_control_manual_enabled) {
                _control_mode.flag_control_manual_enabled = false;
                topic_changed = true;
            }
            //if (_control_mode.flag_control_attitude_enabled) {
            //    _control_mode.flag_control_attitude_enabled = false;
            //   topic_changed = true;
            //}
            if (_control_mode.flag_control_position_enabled) {
                _control_mode.flag_control_position_enabled = false;
                topic_changed = true;
            }
            if (_control_mode.flag_control_velocity_enabled) {
                _control_mode.flag_control_velocity_enabled = false;
                topic_changed = true;
            }
            if (!_control_mode.flag_control_acceleration_enabled) {
                _control_mode.flag_control_acceleration_enabled = true;
                topic_changed = true;
            }
            // publish to vehicle control mode topic if topic is changed
            if (topic_changed) {
                orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_control_mode);
            }

            H = -output_h - 0.06f; //compensate for roll and pitch, and add offset, in D direction
            vel_z_sp = k_p.get() * (-h_sp.get() - H);//h_sp k_p param;
            vel_z = _h_deriv.update(H);
            acc_z = _vel_z_deriv.update(vel_z);
            float vel_err_z = vel_z_sp - vel_z;
            float thrust_desired_D = k_vel_p.get() * vel_err_z + k_vel_d.get() * acc_z + _thr_int_z - MPC_THR_HOVER.get(); //k_vel_p, k_vel_d
            float uMax = -MPC_THR_MIN.get();
            float uMin = -MPC_THR_MAX.get();
            bool stop_integral_D = (thrust_desired_D >= uMax && vel_err_z >= 0.0f) ||
                           (thrust_desired_D <= uMin && vel_err_z <= 0.0f);
            //PX4_INFO("stop_integral_D:\t%8.4f\t", (double)stop_integral_D);
            if (!stop_integral_D){
                _thr_int_z += vel_err_z * k_vel_i.get() * _dt;        //k_vel_i

                //PX4_INFO("_thr_int_z:\t%8.4f\t", (double)_thr_int_z);
                // limit thrust integral
                _thr_int_z = math::min(fabsf(_thr_int_z), MPC_THR_MAX.get()) * math::sign(_thr_int_z);

                //PX4_INFO("_thr_int_z88:\t%8.4f\t", (double)_thr_int_z);
            }
                //PX4_INFO("thrust_desired_D:\t%8.4f\t", (double)thrust_desired_D);
            _thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);
            //PX4_INFO("H:\t%8.4f\tvel_z_sp:\t%8.4f\tvel_err_z:\t%8.4f\thrust_desired_D:\t%8.4f\t", (double)H,(double)vel_z_sp,(double)vel_err_z,(double)thrust_desired_D);
            /*thrust in perching and non perching direction */
            //PX4_INFO("H:\t%8.4f\tthrust_desired_D:\t%8.4f\tthrust_setpoint_D:\t%8.4f\t", (double)H,(double)thrust_desired_D,(double)_thr_sp(2));
            float motion_err_y = 0 - xmotion;
            float acc_y = _xmotion_deriv.update(xmotion);
            float ofd_err = desired_ofd.get() - flow_divergence;
            
            // PID-velocity controller for NE-direction.
            Vector2f thrust_desired_xy;
            thrust_desired_xy(0) = k_ofd_p.get() * (ofd_err); //kofd_p
            thrust_desired_xy(1) = k_m_p.get()* motion_err_y + k_m_d.get() * acc_y + _thr_int_y; //k_m_p, k_m_d, k_m_i

            // Get maximum allowed thrust in NE based on tilt and excess thrust.
            float thrust_max_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
            float thrust_max = sqrtf(MPC_THR_MAX.get() * MPC_THR_MAX.get() - _thr_sp(2) * _thr_sp(2));
            thrust_max = math::min(thrust_max_tilt, thrust_max);

            // Saturate thrust in NE-direction.
            _thr_sp(0) = thrust_desired_xy(0);
            _thr_sp(1) = thrust_desired_xy(1);

            if (thrust_desired_xy * thrust_desired_xy > thrust_max* thrust_max) {
                float mag = thrust_desired_xy.length();
                _thr_sp(0) = thrust_desired_xy(0) / mag * thrust_max;
                _thr_sp(1) = thrust_desired_xy(1) / mag * thrust_max;
            }

            // Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
            // see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
            float arw_gain = 2.f / k_m_p.get();

            float vel_err_lim;
            vel_err_lim = motion_err_y - (thrust_desired_xy(1) - _thr_sp(1)) * arw_gain;

            // Update integral
            _thr_int_y += k_m_i.get() * vel_err_lim * _dt;  //k_m_i

            //PX4_INFO("motion_err_y:\t%8.4f\tacc_y:\t%8.4f\tthrust_desired_nop:\t%8.4f\tthrust_setpoint_nop:\t%8.4f\t",(double)motion_err_y,(double)acc_y,(double)thrust_desired_xy(1),(double)_thr_sp(1));
            //PX4_INFO("ofd_err:\t%8.4f\tflow_divergence:\t%8.4f\tthrust_desired_ofd:\t%8.6f\tthrust_setpoint_perch:\t%8.4f\t", (double)ofd_err,(double)flow_divergence,(double)thrust_desired_xy(0),(double)_thr_sp(0));
            /*yaw setpoint control */
            //PX4_INFO("thrust_setpoint:\t%8.4f\t%8.4f\t%8.4f", (double)_thr_sp(0),(double)_thr_sp(1),(double)_thr_sp(2));
            yaw_sp = yaw + direction_guidence * k_yaw.get(); //k_yaw;
            yaw_rate_sp = k_yaw_rate.get() * direction_guidence ; //k_yaw_rate


            _att_sp = ControlMath_Perch::thrustToAttitude(_thr_sp, yaw_sp, yaw);
            _att_sp.yaw_sp_move_rate = yaw_rate_sp;
            _att_sp.fw_control_yaw = false;
            _att_sp.apply_flaps = false;
            _att_sp.thrust_body[1] = thrust_desired_xy(0);

            _att_sp.timestamp = hrt_absolute_time();

            if (_att_sp_pub != nullptr) {
                orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

             } else if (_attitude_setpoint_id) {
                  _att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
             }
             //PX4_INFO("ATTITUDE_SETPOINT:\t%8.4f\t%8.4f\t%8.4f\t%8.4f", (double)_att_sp.roll_body,(double)_att_sp.pitch_body,(double)_att_sp.yaw_body,(double)_att_sp.thrust_body[2]);
             px4_usleep(10000);

        }

        bool topic_changed = false;

        if (!_control_mode.flag_control_manual_enabled) {
            _control_mode.flag_control_manual_enabled = true;
            topic_changed = true;
        }
        //if (_control_mode.flag_control_attitude_enabled) {
        //    _control_mode.flag_control_attitude_enabled = false;
        //   topic_changed = true;
        //}
        if (_control_mode.flag_control_perch_enabled) {
            _control_mode.flag_control_perch_enabled = false;
            topic_changed = true;
        }
        // publish to vehicle control mode topic if topic is changed
        if (topic_changed) {
            orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_control_mode);
        }
    }
    orb_unsubscribe(optical_front_sub);
    orb_unsubscribe(optical_downward_sub);
    orb_unsubscribe(dis_z_sub);
    orb_unsubscribe(att_sub);
    orb_unsubscribe(vehicle_status_sub);
    orb_unsubscribe(vehicle_control_mode_sub);

    PX4_INFO("exiting");
}

int MulticopterPerchingControl::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("perching_control",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_POSITION_CONTROL,
                       1900,
                       (px4_main_t)&run_trampoline,
                       (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

void MulticopterPerchingControl::updateConstraints(const vehicle_constraints_s &constraints)
{
    _constraints = constraints;

    // For safety check if adjustable constraints are below global constraints. If they are not stricter than global
    // constraints, then just use global constraints for the limits.

    if (!PX4_ISFINITE(constraints.tilt)
        || !(constraints.tilt < math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get()))) {
        _constraints.tilt = math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get());
    }

    if (!PX4_ISFINITE(constraints.speed_up) || !(constraints.speed_up < MPC_Z_VEL_MAX_UP.get())) {
        _constraints.speed_up = MPC_Z_VEL_MAX_UP.get();
    }

    if (!PX4_ISFINITE(constraints.speed_down) || !(constraints.speed_down < MPC_Z_VEL_MAX_DN.get())) {
        _constraints.speed_down = MPC_Z_VEL_MAX_DN.get();
    }

    if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < MPC_XY_VEL_MAX.get())) {
        _constraints.speed_xy = MPC_XY_VEL_MAX.get();
    }
}

MulticopterPerchingControl *MulticopterPerchingControl::instantiate(int argc, char *argv[])
{
    return new MulticopterPerchingControl();
}

int MulticopterPerchingControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int perching_control_main(int argc, char *argv[])
{
    return MulticopterPerchingControl::main(argc, argv);
}