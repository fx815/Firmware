/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_perching_control_params.c
 * Multicopter perching controller parameters.
 *
 * @author Feng Xiao <feng.xiao16@imperial.ac.uk>
 */

/**
 * P gain in Z direction for perching control
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_Z_KP, 1.0f);

/**
 * flying altitude
 *
 * @unit m
 * @min 0.1
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_Z_H, 0.3f);

/**
 * Proportional gain for vertical velocity error for perching control
 *
 * @min 0.01
 * @max 0.4
 * @decimal 2
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_Z_VEL_KP, 0.1f);

/**
 * Integral gain for vertical velocity error for perching control
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_Z_VEL_KI, 0.01f);

/**
 * Differential gain for vertical velocity error for perching control
 *
 * @min 0.0
 * @max 0.1
 * @decimal 3
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_Z_VEL_KD, 0.005f);

/**
 * Proportional gain for perching direction control
 *
 * @min 0.005
 * @max 8.0
 * @decimal 1
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_OFD_KP, 0.03f);

/**
 * Desired Optical flow divergence
 *
 * @min 0.005
 * @max 50.0
 * @decimal 2
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_OFD_GOAL, 10.0f);

/**
 * proportaional gain for non perching direction control
 *
 * @min 0.5
 * @max 4.0
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_NP_KP, 3.0f);

/**
 * Integral gain for non perching direction control
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_NP_KI, 0.1f);

/**
 * Differential gain for non perching direction control
 *
 * @min 0.06
 * @max 0.15
 * @decimal 2
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_NP_KD, 0.1f);

/**
 * Proportional gain for yaw control
 *
 * @min 0.0
 * @max 3.0
 * @decimal 3
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_YAW_KP, 0.02f);

/**
 * Proportional gain for yaw rate control
 *
 * @min 0.005
 * @max 0.1
 * @decimal 3
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_YAW_RATE_KP, 0.01f);

/**
 * Low pass filter cut freq. for numerical height velocity derivative
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(PC_VELZ_LP, 5.0f);

/**
 * Low pass filter cut freq. for numerical height derivative
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(PC_HD_LP, 5.0f);

/**
 * Low pass filter cut freq. for numerical motion derivative
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(PC_MD_LP, 5.0f);

/**
 * Hover thrust
 *
 * Vertical thrust required to hover.
 * This value is mapped to center stick for manual throttle control.
 * With this value set to the thrust required to hover, transition
 * from manual to Altitude or Position mode while hovering will occur with the
 * throttle stick near center, which is then interpreted as (near)
 * zero demand for vertical speed.
 *
 * This parameter is also important for the landing detection to work correctly.
 *
 * @unit norm
 * @min 0.1
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_HOVER, 0.5f);

/**
 * Minimum thrust in auto thrust control
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MIN, 0.12f);

/**
 * Maximum thrust in auto thrust control
 *
 * Limit max allowed thrust
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MAX, 1.0f);

/**
 * Minimum manual thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 * With MC_AIRMODE set to 1, this can safely be set to 0.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MANTHR_MIN, 0.08f);

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode. If higher speeds
 * are commanded in a mission they will be capped to this velocity.
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 12.0f);

/**
 * Maximum vertical descent velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_DN, 1.0f);

/**
 * Maximum vertical ascent velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_UP, 3.0f);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_AIR, 45.0f);

/**
 * Maximal tilt angle in manual or altitude mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_TILT_MAX, 35.0f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_MAX, 200.0f);

/**
 * Thrust curve in Manual Mode
 *
 * This parameter defines how the throttle stick input is mapped to commanded thrust
 * in Manual/Stabilized flight mode.
 *
 * In case the default is used ('Rescale to hover thrust'), the stick input is linearly
 * rescaled, such that a centered stick corresponds to the hover throttle (see MPC_THR_HOVER).
 *
 * Select 'No Rescale' to directly map the stick 1:1 to the output. This can be useful
 * in case the hover thrust is very low and the default would lead to too much distortion
 * (e.g. if hover thrust is set to 20%, 80% of the upper thrust range is squeezed into the
 * upper half of the stick range).
 *
 * Note: in case MPC_THR_HOVER is set to 50%, the modes 0 and 1 are the same.
 *
 * @value 0 Rescale to hover thrust
 * @value 1 No Rescale
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_THR_CURVE, 0);
