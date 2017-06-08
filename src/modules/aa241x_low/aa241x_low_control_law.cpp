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

#include "perms.c"

const float PI = 3.1415927;

const int N_PERMS = 120;
const int N_WAYPOINTS = 5;
int N_VALID_WAYPOINTS = 5;

float mission_n[N_WAYPOINTS] = {200.0f, 0.0f, 100.0f, -100.0f, 0.0};
float mission_e[N_WAYPOINTS] = {0.0f, 0.0f, -50.0f, -100.0f, -35.0f};
float mission_r[N_WAYPOINTS] = {60.0f,50.0f,60.0f,50.0f,60.0f};
float path_n[N_WAYPOINTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float path_e[N_WAYPOINTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float path_r[N_WAYPOINTS] = {-1,-1,-1,-1,-1};
int current_wp = 0;
int prev_wp = -1;



float wrap2Pi(float angleRad) {
	while (angleRad > PI) {
		angleRad -= 2*PI;
	}
	while (angleRad < -PI) {
		angleRad += 2*PI;
	}
	return angleRad;
}

float path_cost(int perm_i) {
  float alpha = 10.0f;
  float cost = 0;
  float n = position_N;
  float e = position_E;
  float chi = ground_course;


  for (int i=0; i<N_WAYPOINTS; i++) {
    int wp_idx = perms[perm_i][i];
    if (mission_r[wp_idx] < 0) {
      continue;
    }

    float next_n = mission_n[wp_idx];
    float next_e = mission_e[wp_idx];
    float next_chi = atan2f(next_e - e, next_n - n);

    float dist = sqrt(pow(next_n - n, 2) + pow(next_e - e, 2));
    float angle = wrap2Pi(next_chi - chi);

    cost += dist + alpha*angle;

    n = next_n;
    e = next_e;
    chi = next_chi;
  }

  return cost;
}

int plan_path(float * mission_n_, float * mission_e_, float * mission_r_, float * path_n_, float * path_e_, float * path_r_)
{
  int best_perm_i = -1;
  float best_cost = 10000.0f;
  for (int i = 0; i < N_PERMS; i++) {
    float cost = path_cost(i);
    if (cost < best_cost) {
      best_cost = cost;
      best_perm_i = i;
    }
  }

  int path_idx = 0;
  for (int j = 1; j < N_WAYPOINTS; j++) {
    int idx = perms[best_perm_i][j];

    if (mission_r_[idx] > 0) {
      path_n[path_idx] = mission_n_[idx];
      path_e[path_idx] = mission_e_[idx];
      path_r[path_idx] = mission_r_[idx];
      path_idx += 1;
    }
  }

  return path_idx;
}

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
  // update mission waypoints in case they have changed
  bool mission_changed = false;
  for(int i=0; i < N_WAYPOINTS; i++){
    if (abs(mission_n[i] - plume_N[i]) > 1e-3f) {
      mission_changed = true;
    }
    mission_n[i] = plume_N[i];
    mission_e[i] = plume_E[i];
    mission_r[i] = plume_radius[i];
  }

  // if mission has changed, reset current wp, replan
  if (mission_changed) {
    current_wp = 0;
    prev_wp = -1;

    N_VALID_WAYPOINTS = plan_path(mission_n, mission_e, mission_r, path_n, path_e, path_r);
  }

  // Check if we've reached the waypoint, or overshot, and increment goal if we have
  float n_error = path_n[current_wp] - position_N;
  float e_error = path_e[current_wp] - position_E;

  if ( pow(n_error,2) + pow(e_error,2) < pow(path_r[current_wp],2) ) {
    low_data.field7 = current_wp;
    current_wp += 1;
    prev_wp += 1;
  } else if ( high_data.field10 < 0 ) {
    current_wp += 1;
    prev_wp += 1;
  }

  current_wp = current_wp % N_VALID_WAYPOINTS;
  prev_wp = prev_wp % N_VALID_WAYPOINTS;

  if (prev_wp >= 0) {
    low_data.field1 = path_n[prev_wp];
    low_data.field2 = path_e[prev_wp];
    low_data.field3 = 70;
  } else {
    low_data.field1 = 0;
    low_data.field2 = 0;
    low_data.field3 = -1;
  }
  low_data.field4 = path_n[current_wp];
  low_data.field5 = path_e[current_wp];
  low_data.field6 = 70;

}
