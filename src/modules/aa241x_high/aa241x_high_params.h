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
 * @file aa241x_fw_control_params.h
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */
#pragma once

#ifndef AA241X_FW_CONTROL_PARAMS_H_
#define AA241X_FW_CONTROL_PARAMS_H_


#include <systemlib/param/param.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Struct of all of the custom parameters.
 *
 * Please make sure to add a variable for each of your newly defined
 * parameters here.
 */
struct aah_params {

	// Throttle loop
	float k_throttle_p;
	float k_throttle_i;
	// Pitch loop
	float k_elev_p;
	float k_elev_d;
	// Altitude loop
	float k_alt_p;
	float k_alt_i;
	// Airspeed loop
	float k_vel_p;
	float k_vel_i;

	// Roll loop
	float k_roll_p;
	float k_roll_i;
	// Course loop
	float k_course_p;
	float k_course_i;
	// Sideslip loop
	float k_sideslip_p;
	float k_sideslip_i;

	// TODO: add custom parameter variable names here......

};


/**
 * Struct of handles to all of the custom parameters.
 *
 *  Please make sure to add a variable for each of your newly
 *  defined parameters here.
 *
 *  NOTE: these variable names can be the same as the ones above
 *  (makes life easier if they are)
 */
struct aah_param_handles {

	// Throttle loop
	param_t k_throttle_p;
	param_t k_throttle_i;
	// Pitch loop
	param_t k_elev_p;
	param_t k_elev_d;
	// Altitude loop
	param_t k_alt_p;
	param_t k_alt_i;

	// Roll loop
	param_t k_roll_p;
	param_t k_roll_i;
	// Course loop
	param_t k_course_p;
	param_t k_course_i;
	// Sideslip loop
	param_t k_sideslip_p;
	param_t k_sideslip_i;


	// TODO: add custom parameter variable names here.......

};

/**
 * Initialize all parameter handles and values
 *
 */
int aah_parameters_init(struct aah_param_handles *h);

/**
 * Update all parameters
 *
 */
int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p);

#ifdef __cplusplus
}
#endif




#endif /* AA241X_FW_CONTROL_PARAMS_H_ */
