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
#include <math.h>

// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)
float altitude_desired = 0.0f;
float vel_desired = 0.0f;

const float PI = 3.1415927;

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

float PIDController::tick(float y_c, float y, float u_ff, float kp, float ki, float kd, bool flag)
{
	float error = y_c - y;
	integrator_ += (Ts_/2)*(error + error_d1_);
	differentiator_ = (2*tau_ - Ts_)/(2*tau_ + Ts_) * differentiator_
										+ 2/(2*tau_ + Ts_)*(error - error_d1_);
	error_d1_ = error;

	float u_unsat = kp*error + ki*integrator_ + kd*differentiator_ + u_ff;
	float u = sat(u_unsat);

	// anti windup
	if (abs(ki) > 1e-7) {
		integrator_ += Ts_/ki * (u - u_unsat);
	}

	return u;
}

float PIDController::tick(float error, float u_ff, float kp, float ki, float kd, bool flag)
{
	integrator_ += (Ts_/2)*(error + error_d1_);
	differentiator_ = (2*tau_ - Ts_)/(2*tau_ + Ts_) * differentiator_
										+ 2/(2*tau_ + Ts_)*(error - error_d1_);
	error_d1_ = error;

	float u_unsat = kp*error + ki*integrator_ + kd*differentiator_ + u_ff;
	float u = sat(u_unsat);

	// anti windup
	if (abs(ki) > 1e-7f) {
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

PathFollower::PathFollower()
	: start_n_(0), end_n_(0), start_e_(0), end_e_(0), start_h_(0), end_h_(0)
{
	// nothing to do
}

void PathFollower::setPath(float start_n, float start_e, float start_h, float end_n, float end_e, float end_h)
{
	start_n_ = start_n;
	start_e_ = start_e;
	start_h_ = start_h;

	end_n_ = end_n;
	end_e_ = end_e;
	end_h_ = end_h;

	q_.n = end_n - start_n;
	q_.e = end_e - start_e;
	q_.d = start_h - end_h;

	chi_q_ = atan2(q_.e, q_.n);
	path_length_ = sqrt(pow(q_.n, 2) + pow(q_.e,2));
}

control_command_t PathFollower::tick(float n, float e, float h, float chi_inf, float k_path, float speed)
{
	ned_t error = {n - start_n_, e - start_e_, start_h_ - h};
	float ey = cosf(chi_q_)*error.e - sinf(chi_q_)*error.n;
	float ex = cosf(chi_q_)*error.n + sinf(chi_q_)*error.e;

	float chi_c = chi_q_ - chi_inf * 2 / PI * atanf(k_path*ey);
	float distance_left = path_length_ - ex;
	control_command_t command = {end_h_, chi_c, speed, distance_left};
	return command;
}

PIDController roll_controller(1.0f, 0.017f, 1.0f); // output = roll_servo
PIDController course_controller(1.0f, 0.017f, 0.785f); // output = commanded roll
PIDController sideslip_controller(1.0f, 0.017f, 1.0f); // output = yaw_servo
PIDController throttle_controller(1.0f, 0.017f, 0.0f, 1.0f); // output = throttle_servo
PIDController pitch_controller(1.0f, 0.017f, 1.0f); // output = pitch_servo
PIDController altitude_controller(1.0f, 0.017f, 0.57f); // output = commanded pitch

PathFollower pathFollower;

float normalizeAngle(float angleRad) {
	while (angleRad > PI) {
		angleRad -= 2*PI;
	}
	while (angleRad < -PI) {
		angleRad += 2*PI;
	}
	return angleRad;
}

float start_n;
float start_e;
float start_h;

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
		altitude_desired = -position_D_gps; 		// altitude_desired needs to be declared outside flight_control() function
		vel_desired = ground_speed;

		// Initiate Path Follower's Path
		start_n = position_N;
		start_e = position_E;
		start_h = -position_D_gps;
	}

	if (low_data.field3 < 0.0f) {
		pathFollower.setPath(start_n, start_e, start_h, low_data.field4, low_data.field5, low_data.field6);
	} else {
		pathFollower.setPath(low_data.field1, low_data.field2, low_data.field3, low_data.field4, low_data.field5, low_data.field6);
	}

	// pathFollower.setPath(start_n, start_e, start_h, aah_parameters.goal_n, aah_parameters.goal_e, aah_parameters.alt_des);


	float AIRSPEED = 17.0f;
	control_command_t command = pathFollower.tick(position_N, position_E, -position_D_gps, aah_parameters.chi_inf, aah_parameters.k_path, AIRSPEED);

	// vel_desired = command.speed;
	// altitude_desired = command.altitude;
	// yaw_desired = command.course;

	// if (phase_num == 4) {
	// 	altitude_desired = 50.0f;
	// }

	if (aah_parameters.alt_des > 0.5f) {
		altitude_desired = aah_parameters.alt_des;
	}
	if (aah_parameters.course_des > 0) {
	  yaw_desired = aah_parameters.course_des;
	}

	high_data.field7 = yaw_desired;
	high_data.field10 = command.distance_left; // to allow lower level logic to determine when to switch waypoints.

	// throtttle controller
	float k_vel_p = aah_parameters.k_throttle_p;
	float throttle_ff = aah_parameters.throttle_ff_b + aah_parameters.throttle_ff_m*vel_desired; //0.625f + 0.0075f*vel_desired;
	throttle_servo_out = throttle_controller.tick(vel_desired, ground_speed, throttle_ff, k_vel_p, 0, 0, false);


	// altitude controller
	float k_alt_p = aah_parameters.k_alt_p;
	pitch_desired = altitude_controller.tick(altitude_desired, -position_D_gps, 0, k_alt_p, 0, 0, false);
	high_data.field1 = pitch_desired;
	// pitch controller
	// pitch_desired = 0.75f*man_pitch_in;
	float kp_pitch = aah_parameters.k_elev_p;
	float pitch_error = normalizeAngle(pitch_desired - pitch);
	pitch_servo_out = pitch_controller.tick(pitch_error, 0, kp_pitch, 0, 0, false);


	// course controller
	float kp_course = aah_parameters.k_course_p;
	float course_error = normalizeAngle(yaw_desired - ground_course);
	roll_desired = course_controller.tick(course_error, 0, kp_course, 0, 0, false);
	high_data.field2 = roll_desired;
	// roll controller
	// roll_desired = -0.75f*man_roll_in;
	float kp_roll = aah_parameters.k_roll_p;
	float roll_error = normalizeAngle(roll_desired - roll);
	roll_servo_out = -roll_controller.tick(roll_error, aah_parameters.roll_offset, kp_roll, 0, 0, false);

	// sideslip controller
	float kp_slip = aah_parameters.k_sideslip_p;
	float sideslip = speed_body_v / (ground_speed + 1e-5f);
	high_data.field3 = sideslip;
	yaw_servo_out = sideslip_controller.tick(0, sideslip, 0, kp_slip, 0, 0, false);

	// getting low data value example
	// float my_low_data = low_data.field1;
	high_data.field4 = yaw_desired;
	high_data.field5 = altitude_desired;
	high_data.field6 = vel_desired;

	// ENSURE THAT YOU SET THE SERVO OUTPUTS!!!
	// outputs should be set to values between -1..1 (except throttle is 0..1)
	// where zero is no actuation, and -1,1 are full throw in either the + or - directions

	// roll_servo_out = man_roll_in;
	// pitch_servo_out = -man_pitch_in;
	// yaw_servo_out = man_yaw_in;
	// throttle_servo_out = man_throttle_in;
}
