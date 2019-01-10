/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#ifndef AWESOME_H_
#define AWESOME_H_

#include <px4.h>
#include <px4_app.h>
#include <px4_module.h>
#include <px4_module_params.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>


extern "C" __EXPORT int awesome_main(int argc, char *argv[]);


class Awesome : public ModuleBase<Awesome>, public ModuleParams {

  public:
	Awesome(int example_param, bool example_flag);
	
    virtual ~Awesome() = default;

	static int task_spawn(int argc, char *argv[]);

	static Awesome *instantiate(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	void run() override;

	int print_status() override;

    void laugh();

  private:

	// Check for parameter changes and update them if needed.
	// @param parameter_update_sub uorb subscription to parameter_update
	// @param force for a parameter update
	void parameters_update(int parameter_update_sub, bool force = false);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,  // < example parameter
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _sys_autoconfig // < another parameter
	)

	// Messages
	struct sensor_combined_s    _sensor_combined_msg;
	struct vehicle_command_s    _vehicle_command_msg;
	struct vehicle_local_position_setpoint_s  _vehicle_local_position_setpoint_msg;
	struct position_setpoint_triplet_s  _position_setpoint_triplet_msg;

    // Subscriptions
	int _parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int _sensor_combined_sub   = orb_subscribe(ORB_ID(sensor_combined));
	int _vehicle_command_sub   = orb_subscribe(ORB_ID(vehicle_command));
	int _vehicle_local_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int _position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

	// Publications
	// orb_advert_t _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_vehicle_command_pub_msg);
	// orb_advert_t _position_setpoint_pub = orb_advertise(ORB_ID(position_setpoint), &_position_setpoint_msg);;
	orb_advert_t _position_setpoint_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_position_setpoint_triplet_msg);;

	// Rate
	px4::Rate _loop_rate;

};

#endif // AWESOME_H_
