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
 * @file aa241x_fw_control.h
 *
 * Header file for student's fixedwing control law.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */
#pragma once

#ifndef AA241X_FW_CONTROL_MAIN_H_
#define AA241X_FW_CONTROL_MAIN_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

// TODO: write your function prototypes here
class PIDController
{
  float integrator_;
  float differentiator_;
  float error_d1_;

  float tau_; // differentiator constant
  float Ts_;
  float upper_limit_;
  float lower_limit_;

public:
  PIDController(float tau, float Ts, float limit);
  PIDController(float tau, float Ts, float lower, float upper);


  float tick(float y_c, float y, float u_ff, float kp, float ki, float kd, bool flag);
  float tick(float error, float u_ff, float kp, float ki, float kd, bool flag);
  float sat(float x);
};

typedef struct {
  float altitude;
  float course;
  float speed;
} control_command_t;

typedef struct {
  float n;
  float e;
  float d;
} ned_t;

class PathFollower
{
  float start_n_;
  float end_n_;
  float start_e_;
  float end_e_;
  float start_h_;
  float end_h_;

  ned_t q_;
  float chi_q_;
public:
  PathFollower();
  void setPath(float start_n, float start_e, float start_h, float end_n, float end_e, float end_h);
  control_command_t tick(float n, float e, float h, float chi_inf, float k_path, float speed);
};



#endif /* AA241X_FW_CONTROL_MAIN_H_ */
