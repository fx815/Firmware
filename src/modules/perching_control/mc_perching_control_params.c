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
PARAM_DEFINE_FLOAT(PC_Z_KP, 0.12f);

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
PARAM_DEFINE_FLOAT(PC_Z_H, 0.5f);

/**
 * Proportional gain for vertical velocity error for perching control
 *
 * @min 0.1
 * @max 0.4
 * @decimal 2
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_Z_VEL_KP, 0.2f);

/**
 * Integral gain for vertical velocity error for perching control
 *
 * @min 0.01
 * @max 0.1
 * @decimal 3
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_Z_VEL_KI, 0.02f);

/**
 * Differential gain for vertical velocity error for perching control
 *
 * @min 0.0
 * @max 0.1
 * @decimal 3
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_Z_VEL_KD, 0.0f);

/**
 * Proportional gain for perching direction control
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @decimal 1
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_OFD_KP, 3.0f);

/**
 * Desored Optical flow divergence
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
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_NP_KP, 1.0f);

/**
 * Integral gain for non perching direction control
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_NP_KI, 0.9f);

/**
 * Differential gain for non perching direction control
 *
 * @min 0.06
 * @max 0.15
 * @decimal 2
 * @group Multicopter Perching Control
 */
PARAM_DEFINE_FLOAT(PC_NP_KD, 0.09f);

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

