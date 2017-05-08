/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

/*
 * @file aa241x_fw_control.cpp
 *
 * Secondary file to the fixedwing controller containing the
 * control law for the AA241x class.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)
float altitude_desired = 0.0f;

/**
 * PIDController class
 */
PIDController::PIDController(float tau, float Ts, float limit)
	: integrator_(0), differentiator_(0), error_d1_(0),
	  tau_(tau), Ts_(Ts), upper_limit_(limit), lower_limit_(-limit)
{
	// nothing else to do
}

PIDController::PIDController(float tau, float Ts, float lower, float upper)
	: integrator_(0), differentiator_(0), error_d1_(0),
	  tau_(tau), Ts_(Ts), upper_limit_(upper), lower_limit_(lower)
{
	// nothing else to do
}

float PIDController::tick(float y_c, float y, float kp, float ki, float kd, bool flag)
{
	float error = y_c - y;
	integrator_ += (Ts_/2)*(error + error_d1_);
	differentiator_ = (2*tau_ - Ts_)/(2*tau_ + Ts_) * differentiator_
										+ 2/(2*tau_ + Ts_)*(error - error_d1_);
	error_d1_ = error;

	float u_unsat = kp*error + ki*integrator_ + kd*differentiator_;
	float u = sat(u_unsat);

	// anti windup
	if (abs(ki) > 1e-7) {
		integrator_ += Ts_/ki * (u - u_unsat);
	}

	return u;
}

float PIDController::sat(float x) {
	if (x > upper_limit_) {
		return upper_limit_;
	} else if (x < lower_limit_) {
		return lower_limit_;
	} else {
		return x;
	}
}

PIDController roll_controller(1.0f, 0.017f, 1.0f); // output = roll_servo
PIDController course_controller(1.0f, 0.017f, 0.57f); // output = commanded roll
PIDController sideslip_controller(1.0f, 0.017f, 1.0f); // output = yaw_servo
PIDController throttle_controller(1.0f, 0.017f, 0.0f, 1.0f); // output = throttle_servo
PIDController pitch_controller(1.0f, 0.017f, 1.0f); // output = pitch_servo
PIDController altitude_controller(1.0f, 0.017f, 0.57f); // output = commanded pitch



/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */
void flight_control() {

	// An example of how to run a one time 'setup' for example to lock one's altitude and heading...
	if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
																	 //	should only occur on first engagement since this is 59Hz loop
		yaw_desired = yaw; 							// yaw_desired already defined in aa241x_high_aux.h
		altitude_desired = position_D_baro; 		// altitude_desired needs to be declared outside flight_control() function
	}


	// TODO: write all of your flight control here...
	// throtttle controller
	// float vel_desired = 20.0f*man_throttle_in;
	// float k_vel_p = aah_parameters.k_vel_p;
	// throttle_servo_out = throttle_controller.tick(vel_desired, ground_speed, k_vel_p, 0, 0, false);

	// altitude controller
	// float k_alt_p = aah_parameters.k_alt_p;
	// pitch_desired = altitude_controller.tick(altitude_desired, position_D_baro, k_alt_p, 0, 0, false);


	// pitch controller
	pitch_desired = man_pitch_in;
	float kp_pitch = aah_parameters.k_elev_p;
	pitch_servo_out = pitch_controller.tick(pitch_desired, pitch, kp_pitch, 0, 0, false);

	// roll controller
	roll_desired = -man_roll_in;
	float kp_roll = aah_parameters.k_roll_p;
	roll_servo_out = roll_controller.tick(roll_desired, roll, -kp_roll, 0, 0, false);

	// getting low data value example
	// float my_low_data = low_data.field1;


	// ENSURE THAT YOU SET THE SERVO OUTPUTS!!!
	// outputs should be set to values between -1..1 (except throttle is 0..1)
	// where zero is no actuation, and -1,1 are full throw in either the + or - directions

	// roll_servo_out = man_roll_in;
	// pitch_servo_out = -man_pitch_in;
	yaw_servo_out = man_yaw_in;
	throttle_servo_out = man_throttle_in;
}
