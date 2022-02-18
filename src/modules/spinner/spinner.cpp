/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file spinner.cpp
 * Controller prototype for single rotor drone
 *
 * @author Evandro Bernardes <evandro.bernardes@univ-amu.fr>
 */

#include "spinner.hpp"
#include "spinner_helpers.cpp"

/* Main Thread */
static int spinner_thread_main(int argc, char *argv[])
{
  /* welcome user (warnx prints a line, including an appended\n, with variable arguments */
  PX4_INFO("started");

  if(init_system() != 0){
    thread_should_exit = true;
    return 1;
  } else {
    thread_running = true;
  }

	while (!thread_should_exit) {

		// Wait for a sensor or param update, check for exit condition every 500 ms.
		int ret = poll(fds, 1, 10);
		if (ret < 0) {
			warnx("poll error");
		} else if (ret == 0) {
			/* no return value = nothing changed for 100 ms, ignore */
		} else {


			if (fds[0].revents & POLLIN) {/* only run controller if attitude changed */
        read_topics(); // reads manual input, angular velocity and actuator
        set_att_message();
        control_attitude();
        publish_actuators();
			}
		}
	}

	PX4_INFO("exiting, stopping controller.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuator_control.control[i] = 0.0f;
	}
  actuator_control.control[4] = 1.0f;
  publish_actuators();
	fflush(stdout);
	return 0;
}

/* Startup Functions */

static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: spinner {start|stop|status|switch|param|thrust|goal}\n\n");
}

/**
 * The daemon app only briefly exists to start the background job.
 */
int spinner_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

  if (!strcmp(argv[1], "thrust")) {
    if (argc < 3) {
      PX4_INFO("usage: spinner thrust {spin|constant}\n");
      return -1;

    } else if (!thread_running) {
      PX4_INFO("Thread not running\n");
      return -1;

    } else if (!strcmp(argv[2], "spin")) {
      if (thrust_mode == THRUST_SPIN) {
        PX4_INFO("already in SPIN thrust mode\n");
      } else {
        thrust_mode = THRUST_SPIN;
        PX4_INFO("thrust mode changed to SPIN\n");
      }

    } else if (!strcmp(argv[2], "constant")) {
      if (thrust_mode == THRUST_CONSTANT) {
        PX4_INFO("already in CONSTANT thrust mode\n");
      } else {
        thrust_mode = THRUST_CONSTANT;
        PX4_INFO("thrust mode changed to CONSTANT\n");
      }

    } else {
      PX4_INFO("usage: spinner thrust {spin|constant}\n");
      return -1;
    }

		return 0;
  }

  if (!strcmp(argv[1], "goal")) {
    if (argc < 3) {
      PX4_INFO("usage: spinner goal {manual|setpoint}\n");
      return -1;

    } else if (!thread_running) {
      PX4_INFO("Thread not running\n");
      return -1;

    } else if (!strcmp(argv[2], "manual")) {
      if (goal_mode == GOAL_MANUAL) {
        PX4_INFO("already in MANUAL goal mode\n");
      } else {
        goal_mode = GOAL_MANUAL;
        PX4_INFO("goal mode changed to MANUAL\n");
      }

    } else if (!strcmp(argv[2], "setpoint")) {
      if (goal_mode == GOAL_SETPOINT) {
        PX4_INFO("already in SETPOINT goal mode\n");
      } else {
        goal_mode = GOAL_SETPOINT;
        PX4_INFO("goal mode changed to SETPOINT\n");
      }

    } else {
      PX4_INFO("usage: spinner goal {manual|setpoint}\n");
      return -1;
    }

		return 0;
  }

  if (!strcmp(argv[1], "switch")) {
    if (argc < 3) {
      PX4_INFO("usage: spinner switch {enu|ned}\n");
      return -1;

    } else if (!thread_running) {
      PX4_INFO("Thread not running\n");
      return -1;

    } else if (!strcmp(argv[2], "enu")) {
      if (quat_mode == QUAT_ENU) {
        PX4_INFO("already in ENU frame\n");
      } else {
        quat_mode = QUAT_ENU;
        PX4_INFO("frame changed to ENU\n");
      }

    } else if (!strcmp(argv[2], "ned")) {
      if (quat_mode == QUAT_NED) {
        PX4_INFO("already in NED frame\n");
      } else {
        quat_mode = QUAT_NED;
        PX4_INFO("frame changed to NED\n");
      }

    } else if (!strcmp(argv[2], "null")) {
      if (quat_mode == QUAT_NULL) {
        PX4_INFO("already in null rotation mode\n");
      } else {
        quat_mode = QUAT_NULL;
        PX4_INFO("frame changed identity rotation and zero angular velocity\n");
      }

    } else {
      PX4_INFO("usage: spinner switch {enu|ned|null}\n");
      return -1;
    }

		return 0;
  }

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("spinner already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("spinner",
						 SCHED_DEFAULT,
						//  SCHED_PRIORITY_SLOW_DRIVER,
						 SCHED_PRIORITY_ATTITUDE_CONTROL,
						 2048,
						 spinner_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		thread_running = true;
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "debug")) {
		if (is_debug){
      PX4_INFO("Toggling debug OFF\n");
      is_debug = false;
    } else {
      PX4_INFO("Toggling debug ON\n");
      is_debug = true;
    }
		return 0;
	}

	if (!strcmp(argv[1], "param")) {
    if (argc < 3) {
      // PX4_INFO("usage: spinner param {load|print|set}\n");
      PX4_INFO("usage: spinner param {load|print}\n");
      return -1;

    } else if (!strcmp(argv[2], "load")) {
      parameter_load_all(params, N_PARAMETERS, true);
      Binv = Id * params[PROP_K_RATIO].value + e.hat() * params[PROP_HEIGHT].value;
      Binv.I() = Binv.I();

    } else if (!strcmp(argv[2], "print")) {
      parameter_print_all(params, N_PARAMETERS);

    } else if (!strcmp(argv[2], "set")) {
      if (argc < 5) {
        PX4_INFO("usage: spinner param set [P / D / S / C] [new K value]\n");
        return -1;
      }
      if(strcmp(argv[3], "P"))
        params[SPIN_KP].value = atof(argv[4]);
      else if(strcmp(argv[3], "D"))
        params[SPIN_KD].value = atof(argv[4]);
      else if(strcmp(argv[3], "S"))
        params[SPIN_KS].value = atof(argv[4]);
      else if(strcmp(argv[3], "C"))
        params[SPIN_KC].value = atof(argv[4]);
      else{
        PX4_INFO("usage: spinner param set [P / D / S / C] [new K value]\n");
        return -1;
      }

    } else {
      PX4_INFO("usage: spinner param {load|print}\n");
      return -1;
    }
    return 0;
  }

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			PX4_INFO("spinner is running");
      if(quat_mode == QUAT_ENU){
        PX4_INFO("quat_mode = ENU");
      } else if(quat_mode == QUAT_NED){
        PX4_INFO("quat_mode = NED");
      }

      if(goal_mode == GOAL_MANUAL){
        PX4_INFO("goal_mode = MANUAL");
      } else if(goal_mode == GOAL_SETPOINT){
        PX4_INFO("goal_mode = SETPOINT");
      }

		} else {
			PX4_INFO("spinner not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 0;
}
