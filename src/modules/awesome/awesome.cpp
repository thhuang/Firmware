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

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>

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

	/* parse CLI arguments */
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
	: ModuleParams(nullptr) {}

void Awesome::run() {

	/* subscribe to the sensor_combined topic */
	int sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_combined_sub_fd, 200);

	/* subscribe to the vehicle_command topic */
	int vehicle_command_sub_fd = orb_subscribe(ORB_ID(vehicle_command));
	
	/* wakeup source(s) */
    px4_pollfd_struct_t fds[] = {
        {.fd = sensor_combined_sub_fd, .events = POLLIN},  // BROKEN if we are using the L3GS20
        {.fd = vehicle_command_sub_fd, .events = POLLIN},
		/* there could be more file descriptors here, in the form like:
		 * {.fd = other_sub_fd, .events = POLLIN},
		 */
    };

	/* initialize parameters */
	int parameter_update_sub_fd = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(parameter_update_sub_fd, true);

    while (!should_exit()) {
		
        /* wait for up to 1000 ms for data (1 Hz) */
		int poll_ret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			PX4_ERR("poll error %d, %d", poll_ret, errno);
			usleep(50000);
			continue;

		} else {
            
            if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				/* copy sensors raw data into local buffer */
			    orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &sensor_combined);
          
                /* print data */
                /*
                 * PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				 * 	     (double)sensor_combined.accelerometer_m_s2[0],
				 * 	     (double)sensor_combined.accelerometer_m_s2[1],
				 * 	     (double)sensor_combined.accelerometer_m_s2[2]);
                 */
            } // if (fds[0].revents & POLLIN)
			
            if (fds[1].revents & POLLIN) {
                /* obtained data for the second file descriptor */
                struct vehicle_command_s vehicle_command;
                orb_copy(ORB_ID(vehicle_command), vehicle_command_sub_fd, &vehicle_command);

                /* print data */
                PX4_INFO("Vehicle Command: %d", (unsigned)vehicle_command.command);
                PX4_INFO("Parameter 1:\t%8.4f", (double)vehicle_command.param1);
                PX4_INFO("Parameter 2:\t%8.4f", (double)vehicle_command.param2);
                PX4_INFO("Parameter 3:\t%8.4f", (double)vehicle_command.param3);
                PX4_INFO("Parameter 4:\t%8.4f", (double)vehicle_command.param4);
                PX4_INFO("Parameter 5:\t%8.4f", (double)vehicle_command.param5);
                PX4_INFO("Parameter 6:\t%8.4f", (double)vehicle_command.param6);
                PX4_INFO("Parameter 7:\t%8.4f", (double)vehicle_command.param7);

            } // if (fds[1].revents & POLLIN)
        
        } // if (poll_ret != 0)

		parameters_update(parameter_update_sub_fd);
	}

	orb_unsubscribe(sensor_combined_sub_fd);
	orb_unsubscribe(vehicle_command_sub_fd);
	orb_unsubscribe(parameter_update_sub_fd);
}

void Awesome::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}

void Awesome::laugh()
{
    PX4_INFO("WaHaHaHa!!!!!");
	PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
			 (double)sensor_combined.accelerometer_m_s2[0],
			 (double)sensor_combined.accelerometer_m_s2[1],
			 (double)sensor_combined.accelerometer_m_s2[2]);
}

int awesome_main(int argc, char *argv[])
{
	return Awesome::main(argc, argv);
}
