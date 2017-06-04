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
 * @file aa241x_low.cpp
 *
 * Secondary control law file for AA241x.  Contains control/navigation
 * logic to be executed with a lower priority, and "slowly"
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */


// include header file
#include "aa241x_low_control_law.h"
#include "aa241x_low_aux.h"

#include <uORB/uORB.h>

using namespace aa241x_low;

const int N_WAYPOINTS = 5;
float mission_n[N_WAYPOINTS] = {200.0f, 0.0f, 100.0f, -100.0f, 0.0};
float mission_e[N_WAYPOINTS] = {0.0f, 0.0f, -50.0f, -100.0f, -35.0f};
float mission_h[N_WAYPOINTS] = {60.0f,50.0f,60.0f,50.0f,60.0f};
int current_wp = 0;
int prev_wp = -1;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 *
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */
void low_loop()
{
	// Check if we've reached the waypoint, or overshot, and increment goal if we have
  float n_error = mission_n[current_wp] - position_N;
	float e_error = mission_e[current_wp] - position_E;

	if ( pow(n_error,2) + pow(e_error,2) < 100.0 ) {
		current_wp += 1;
    prev_wp += 1;
	} else if ( high_data.field10 < 0 ) {
		current_wp += 1;
    prev_wp += 1;
	}

	current_wp = current_wp % N_WAYPOINTS;
  prev_wp = prev_wp % N_WAYPOINTS;

  if (prev_wp >= 0) {
  	low_data.field1 = mission_n[prev_wp];
  	low_data.field2 = mission_e[prev_wp];
  	low_data.field3 = mission_h[prev_wp];
  } else {
    low_data.field1 = 0;
  	low_data.field2 = 0;
  	low_data.field3 = -1;
  }
  low_data.field4 = mission_n[current_wp];
	low_data.field5 = mission_e[current_wp];
	low_data.field6 = mission_h[current_wp];

}
