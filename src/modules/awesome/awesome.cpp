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

#include "awesome.h"

#include <iostream>

#include <drivers/drv_hrt.h>

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <navigator/navigation.h>


extern "C" __EXPORT int awesome_main(int argc, char *argv[]);


int Awesome::print_usage(const char *reason) {

    if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Awesome module that does nothing!

The provided functionality includes:
- Nothing!
- Do not expect too much.

### Implementation
Top secret!

### Examples
Typical usage to start nothing immediately:
$ awesome start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("awesome", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;

}

int Awesome::print_status() {

    PX4_INFO("Running");
	return 0;

}

int Awesome::custom_command(int argc, char *argv[]) {
	
	if (!is_running()) {
		print_usage("not running");
		return -1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "laugh")) {
		get_instance()->laugh();
		return 0;
	}

	return print_usage("unknown command");

}

int Awesome::task_spawn(int argc, char *argv[]) {

    _task_id = px4_task_spawn_cmd(
        "awesome",                   // Process name
	    SCHED_DEFAULT,               // Scheduling type (RR or FIFO)
        SCHED_PRIORITY_DEFAULT,      // Scheduling priority
        1024,                        // Stack size of the new task or thread
        (px4_main_t)&run_trampoline, // Task (or thread) main function
        (char *const *)argv);        // Void pointer to pass to the new task

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Awesome *Awesome::instantiate(int argc, char *argv[]) {

	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
	
        switch (ch) {
		  case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		  case 'f':
			example_flag = true;
			break;

		  case '?':
			error_flag = true;
			break;

		  default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) return nullptr;

	Awesome *instance = new Awesome(example_param, example_flag);

	if (instance == nullptr) PX4_ERR("alloc failed");

	return instance;
}

Awesome::Awesome(int example_param, bool example_flag)
	: ModuleParams(nullptr), _loop_rate(10) {}

void Awesome::run() {

	// limit the update period (ms)
	// orb_set_interval(_sensor_combined_sub,   200); // 5 Hz
	// orb_set_interval(_vehicle_command_sub,   100); // 10 Hz
	orb_set_interval(_vehicle_local_position_setpoint_sub, 500); // 2 Hz

	// wakeup source(s)
    px4_pollfd_struct_t fds[] = {
        {.fd = _sensor_combined_sub,   				 .events = POLLIN}, // BROKEN if we are using the L3GS20
        {.fd = _vehicle_command_sub,   				 .events = POLLIN},
		{.fd = _vehicle_local_position_setpoint_sub, .events = POLLIN},
		{.fd = _position_setpoint_triplet_sub, 		 .events = POLLIN},
		// there could be more file descriptors here, in the form like:
		// {.fd = other_sub_fd, .events = POLLIN},
    };

	// initialize parameters
	parameters_update(_parameter_update_sub, true);

    while (!should_exit()) {
		
        // wait for up to 1000 ms for data (1 Hz)
		int poll_ret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (poll_ret == 0) {
			// this means none of our providers is giving us data
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			// this is undesirable but not much we can do - might want to flag unhappy status
			PX4_ERR("poll error %d, %d", poll_ret, errno);
			usleep(50000);
			continue;

		} else {
            
            if (fds[0].revents & POLLIN) {
				// obtained data for the first file descriptor
				// copy sensors raw data into local buffer
			    // orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined_msg);

			} // if (fds[0].revents & POLLIN)
			
            if (fds[1].revents & POLLIN) {
                // obtained data for the second file descriptor
				// copy vehicle command into local buffer
                orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &_vehicle_command_msg);

                // print data
				// PX4_INFO("vehicle_command");				
                // PX4_INFO("Vehicle Command: %d",  (unsigned)_vehicle_command_msg.command);
				// PX4_INFO("Target System: %d",    (unsigned)_vehicle_command_msg.target_system);
				// PX4_INFO("Target Component: %d", (unsigned)_vehicle_command_msg.target_component);
				// PX4_INFO("Source System: %d",    (unsigned)_vehicle_command_msg.source_system);
				// PX4_INFO("Source Component: %d", (unsigned)_vehicle_command_msg.target_component);
				// PX4_INFO("Confirmation: %d",     (unsigned)_vehicle_command_msg.confirmation);
				// PX4_INFO("From External: %d",        (bool)_vehicle_command_msg.from_external);
                // PX4_INFO("Parameter 1:\t%8.4f",    (double)_vehicle_command_msg.param1);
                // PX4_INFO("Parameter 2:\t%8.4f",    (double)_vehicle_command_msg.param2);
                // PX4_INFO("Parameter 3:\t%8.4f",    (double)_vehicle_command_msg.param3);
                // PX4_INFO("Parameter 4:\t%8.4f",    (double)_vehicle_command_msg.param4);
                // PX4_INFO("Parameter 5:\t%8.4f",    (double)_vehicle_command_msg.param5);
                // PX4_INFO("Parameter 6:\t%8.4f",    (double)_vehicle_command_msg.param6);
                // PX4_INFO("Parameter 7:\t%8.4f",    (double)_vehicle_command_msg.param7);


            } // if (fds[1].revents & POLLIN)

            if (fds[2].revents & POLLIN) {
				// obtained data for the third file descriptor
				// copy vehicle local position setpoint into local buffer
			    orb_copy(ORB_ID(vehicle_local_position_setpoint), _vehicle_local_position_setpoint_sub, &_vehicle_local_position_setpoint_msg);

				// print data
				// PX4_INFO("vehicle_local_position_setpoint");
				// PX4_INFO("x:        \t%8.4f", (double)_vehicle_local_position_setpoint_msg.x);
				// PX4_INFO("y:        \t%8.4f", (double)_vehicle_local_position_setpoint_msg.y);
				// PX4_INFO("z:        \t%8.4f", (double)_vehicle_local_position_setpoint_msg.z);
				// PX4_INFO("yaw:      \t%8.4f", (double)_vehicle_local_position_setpoint_msg.yaw);
				// PX4_INFO("yawspeed: \t%8.4f", (double)_vehicle_local_position_setpoint_msg.yawspeed);
				// PX4_INFO("vx:       \t%8.4f", (double)_vehicle_local_position_setpoint_msg.vx);
				// PX4_INFO("vy:       \t%8.4f", (double)_vehicle_local_position_setpoint_msg.vy);
				// PX4_INFO("vz:       \t%8.4f", (double)_vehicle_local_position_setpoint_msg.vz);
				// PX4_INFO("acc_x:    \t%8.4f", (double)_vehicle_local_position_setpoint_msg.acc_x);
				// PX4_INFO("acc_y:    \t%8.4f", (double)_vehicle_local_position_setpoint_msg.acc_y);
				// PX4_INFO("acc_z:    \t%8.4f", (double)_vehicle_local_position_setpoint_msg.acc_z);
				// PX4_INFO("jerk_x:   \t%8.4f", (double)_vehicle_local_position_setpoint_msg.jerk_x);
				// PX4_INFO("jerk_y:   \t%8.4f", (double)_vehicle_local_position_setpoint_msg.jerk_y);
				// PX4_INFO("jerk_z:   \t%8.4f", (double)_vehicle_local_position_setpoint_msg.jerk_z);
				// PX4_INFO("thrust N: \t%8.4f", (double)_vehicle_local_position_setpoint_msg.thrust[0]);
				// PX4_INFO("thrust E: \t%8.4f", (double)_vehicle_local_position_setpoint_msg.thrust[1]);
				// PX4_INFO("thrust D: \t%8.4f", (double)_vehicle_local_position_setpoint_msg.thrust[2]);
            
			} // if (fds[0].revents & POLLIN)

            if (fds[3].revents & POLLIN) {
				// obtained data for the third file descriptor
				// copy position setpoint triplet into local buffer
			    orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub, &_position_setpoint_triplet_msg);

				// print data
				PX4_INFO("position_setpoint_triplet");
				PX4_INFO("previous.valid: \t%d", (bool)_position_setpoint_triplet_msg.previous.valid);
				PX4_INFO("current.valid:  \t%d", (bool)_position_setpoint_triplet_msg.current.valid);
				PX4_INFO("next.valid:     \t%d", (bool)_position_setpoint_triplet_msg.next.valid);				
				PX4_INFO("previous.type: \t%d", (unsigned)_position_setpoint_triplet_msg.previous.type);
				PX4_INFO("current.type:  \t%d", (unsigned)_position_setpoint_triplet_msg.current.type);
				PX4_INFO("next.type:     \t%d", (unsigned)_position_setpoint_triplet_msg.next.type);
				PX4_INFO("previous.position_valid: \t%d", (bool)_position_setpoint_triplet_msg.previous.position_valid);
				PX4_INFO("current.position_valid:  \t%d", (bool)_position_setpoint_triplet_msg.current.position_valid);
				PX4_INFO("next.position_valid:     \t%d", (bool)_position_setpoint_triplet_msg.next.position_valid);
				PX4_INFO("previous.velocity_valid: \t%d", (bool)_position_setpoint_triplet_msg.previous.velocity_valid);
				PX4_INFO("current.velocity_valid:  \t%d", (bool)_position_setpoint_triplet_msg.current.velocity_valid);
				PX4_INFO("next.velocity_valid:     \t%d", (bool)_position_setpoint_triplet_msg.next.velocity_valid);
				PX4_INFO("previous.x: \t%8.4f", (double)_position_setpoint_triplet_msg.previous.x);
				PX4_INFO("current.x:  \t%8.4f", (double)_position_setpoint_triplet_msg.current.x);
				PX4_INFO("next.x:     \t%8.4f", (double)_position_setpoint_triplet_msg.next.x);				
				PX4_INFO("previous.y: \t%8.4f", (double)_position_setpoint_triplet_msg.previous.y);
				PX4_INFO("current.y:  \t%8.4f", (double)_position_setpoint_triplet_msg.current.y);
				PX4_INFO("next.y:     \t%8.4f", (double)_position_setpoint_triplet_msg.next.y);	
				PX4_INFO("previous.z: \t%8.4f", (double)_position_setpoint_triplet_msg.previous.z);
				PX4_INFO("current.z:  \t%8.4f", (double)_position_setpoint_triplet_msg.current.z);
				PX4_INFO("next.z:     \t%8.4f", (double)_position_setpoint_triplet_msg.next.z);
				PX4_INFO("previous.alt_valid: \t%d", (bool)_position_setpoint_triplet_msg.previous.alt_valid);
				PX4_INFO("current.alt_valid:  \t%d", (bool)_position_setpoint_triplet_msg.current.alt_valid);
				PX4_INFO("next.alt_valid:     \t%d", (bool)_position_setpoint_triplet_msg.next.alt_valid);
				PX4_INFO("previous.alt: \t%8.4f", (double)_position_setpoint_triplet_msg.previous.alt);
				PX4_INFO("current.alt:  \t%8.4f", (double)_position_setpoint_triplet_msg.current.alt);
				PX4_INFO("next.alt:     \t%8.4f", (double)_position_setpoint_triplet_msg.next.alt);	
				PX4_INFO("previous.lat: \t%8.4f", (double)_position_setpoint_triplet_msg.previous.lat);
				PX4_INFO("current.lat:  \t%8.4f", (double)_position_setpoint_triplet_msg.current.lat);
				PX4_INFO("next.lat:     \t%8.4f", (double)_position_setpoint_triplet_msg.next.lat);	
				PX4_INFO("previous.lon: \t%8.4f", (double)_position_setpoint_triplet_msg.previous.lon);
				PX4_INFO("current.lon:  \t%8.4f", (double)_position_setpoint_triplet_msg.current.lon);
				PX4_INFO("next.lon:     \t%8.4f", (double)_position_setpoint_triplet_msg.next.lon);	

			} // if (fds[0].revents & POLLIN)

        } // if (poll_ret != 0)

		parameters_update(_parameter_update_sub);
	}

	orb_unsubscribe(_sensor_combined_sub);
	orb_unsubscribe(_vehicle_command_sub);
	orb_unsubscribe(_parameter_update_sub);

}

void Awesome::parameters_update(int parameter_update_sub, bool force) {

	bool updated;
	struct parameter_update_s parameter_update_msg;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &parameter_update_msg);
	}

	if (force || updated) {
		updateParams();
	}

}

void Awesome::laugh() {

	PX4_INFO("WaHaHaHa!!!!!");
	PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
			 (double)_sensor_combined_msg.accelerometer_m_s2[0],
			 (double)_sensor_combined_msg.accelerometer_m_s2[1],
			 (double)_sensor_combined_msg.accelerometer_m_s2[2]);

	// TODO: try send_vehicle_command()
	// https://github.com/thhuang/Firmware/blob/XD/src/modules/commander/Commander.cpp#L268-L286
	// TODO: try MulticopterPositionControl
	// https://github.com/thhuang/Firmware/blob/XD/src/modules/mc_pos_control/mc_pos_control_main.cpp
	// https://github.com/thhuang/Firmware/blob/XD/src/modules/mc_pos_control/mc_pos_control_main.cpp#L772-L787
	// TODO: try execute_avoidance_waypoint()
	// https://github.com/thhuang/Firmware/blob/XD/src/modules/mc_pos_control/mc_pos_control_main.cpp#L1175-L1190

	struct position_setpoint_triplet_s msg = _position_setpoint_triplet_msg;
	msg.current.type = 1;
	msg.current.yaw_valid = false;
	msg.current.yawspeed_valid = false;
	msg.current.vx = 5;
	orb_publish(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_pub, &msg);

	// _vehicle_command_pub_msg.command = 192;
	// _vehicle_command_pub_msg.target_system = 1;
	// _vehicle_command_pub_msg.target_component = 1;
	// _vehicle_command_pub_msg.source_system = 255;
	// _vehicle_command_pub_msg.source_component = 1;
	// _vehicle_command_pub_msg.confirmation = 0;
	// _vehicle_command_pub_msg.from_external = true;
	// _vehicle_command_pub_msg.param1 =   -1.0000;
	// _vehicle_command_pub_msg.param2 =    1.0000;
	// _vehicle_command_pub_msg.param3 =    0.0000;
	// _vehicle_command_pub_msg.param4 =    0.0000;
	// _vehicle_command_pub_msg.param5 =   47.3973;
	// _vehicle_command_pub_msg.param6 =    8.5440;
	// _vehicle_command_pub_msg.param7 =  489.7320;
	// orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_vehicle_command_pub_msg);

}

int awesome_main(int argc, char *argv[]) {
	return Awesome::main(argc, argv);
}
