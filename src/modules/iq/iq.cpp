/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file iq.cpp
 * Minimal application example for an underactuated propeller vehicle powered by IQinetics motors
 *
 * @author Matthew Piccoli <matt@iqinetics.com>
 * @author Evandro Bernardes <evandro.bernardes@univ-amu.fr>
 */

#include "iq.hpp"
#include "iq_helper_functions.cpp"

int iq_thread_main(int argc, char *argv[])
{
  PX4_INFO("IQinetics Underactuated Propeller Thread Loading");

  // struct motor_temp_s motor_temp_raw;
  memset(&motor_temp_raw, 0, sizeof(motor_temp_raw));
  temp_pub = orb_advertise(ORB_ID(motor_temp), &motor_temp_raw);

  // Load parameters, set UART comm to motor, set ORB topics
  if(init_system(TIMEOUT, SLEEP_TIME) != 0){
    thread_should_exit = true;
    return 1;
  } else {
    // initialize thread variables
    error_counter = 0;
    thread_running = true;
    // int print_loop = 0;
  }

    // main while loop for this thread
    while(!thread_should_exit)
    {
    // handle errors
		int poll_ret = px4_poll(fds, 3, TOPICS_TIME);
		// /* handle the poll result */
		if (poll_ret == 0) {  /* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a %ims", TOPICS_TIME);

		} else if (poll_ret < 0) {  /* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0)
			{
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}
			error_counter++;

		} else {
		    if (fds[0].revents & POLLIN){
                // Get the actuator armed data
                struct actuator_armed_s actuator_arm_raw;
                orb_copy(ORB_ID(actuator_armed), actuator_arm_sub_fd, &actuator_arm_raw);
                is_armed = actuator_arm_raw.armed;
            }

            // get actuator commands and send commands if armed
            if (is_armed) {
                struct actuator_controls_s actuator_raw;
                if(actuator_control_id == ACTUATOR_MANUAL){
                    orb_copy(ORB_ID(actuator_controls_3), actuator_ctrl_manual_sub_fd, &actuator_raw);
                    if (abs(actuator_raw.control[4] - control[4]) < 0.01f){ // control[4] == 0 if manual, == 1 if iq_test thread
                        thrust_diff = actuator_raw.control[3] - control[3];
                        memcpy(control, actuator_raw.control, sizeof control); // updates if control[4] matches
                    }
                    control[1] = -control[1];
                } else if (actuator_control_id == ACTUATOR_PID){
                    orb_copy(ORB_ID(actuator_controls_0), actuator_ctrl_sub_fd, &actuator_raw);
                    memcpy(control, actuator_raw.control, sizeof control);
                    // control[1] = -control[1];
                } else if (actuator_control_id == ACTUATOR_SPINNER){
                    orb_copy(ORB_ID(actuator_controls_spinner), actuator_ctrl_spinner_sub_fd, &actuator_raw);
                    memcpy(control, actuator_raw.control, sizeof control);
                    // control[1] = -control[1];
                    // control[1] = (control[1]-control[0])/2;
                    // control[0] = (control[1]+control[0]);
                }
            }

            // if state changed from armed to unarmed, enter coast mode
            else if(was_armed) enter_coast_mode();
            else usleep(100);
            send_commands(control);

            // update was_armed to keep state changes in track
            was_armed = is_armed;
        }
	}
	PX4_INFO("exiting");
  thread_running = false;

  fflush(stdout);
	return 0;
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int iq_main(int argc, char *argv[]) {
  if (argc < 2) {
    usage("missing command");
    return 1;
  }

  if (!strcmp(argv[1], "start")) {
    if (thread_running) {
      PX4_INFO("already running\n");
      /* this is not an error */
      return 0;
    }

    PX4_INFO("IQinetics Underactuated Propeller Daemon Spawning");

    // Check input arguments
    // char *uart_name1;
    if (argc < 3) {
        uart_name1 = (char *) ("/dev/ttyS1");
        PX4_WARN("No serial port argument given, using default: /dev/ttyS1");
        // return 1;
    } else if (argc > 3) {
        PX4_ERR("Argument error, correct start usage: iq start [port]");
        return 1;
    } else {
        uart_name1 = argv[2];
    }

    thread_should_exit = false;

    daemon_task = px4_task_spawn_cmd("iq",
            SCHED_DEFAULT,
            SCHED_PRIORITY_FAST_DRIVER - 1,
            2000,
            iq_thread_main,
            //(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
            //  (argv) ? (char *const *)&uart_name1 : (char *const *)NULL);
            (char *const *)NULL);
    PX4_INFO("IQinetics Underactuated Propeller Daemon Spawned");

    return 0;
  }


  if (!strcmp(argv[1], "switch")) {
    if (argc < 3) {
      PX4_INFO("usage: iq switch {flight|roll|test}\n");
      return -1;

    } else if (!thread_running) {
      PX4_INFO("Thread not running\n");
      return -1;

    } else if (!strcmp(argv[2], "flight")) {
      if (mode == MODE_FLIGHT) {
        PX4_INFO("already in flight mode\n");
        // return 0;
      } else {
        PX4_INFO("entering flight mode\n");
        mode = MODE_FLIGHT;
        control[4] = 0.0;
        // return 0;
      }

    } else if (!strcmp(argv[2], "roll")) {
      if (mode == MODE_ROLL) {
        PX4_INFO("already in roll mode\n");
        // return 0;
      } else {
        PX4_INFO("entering roll mode\n");
        mode = MODE_ROLL;
        control[4] = 0.0;
        // return 0;
      }

    } else if (!strcmp(argv[2], "test")) {
      if (mode == MODE_TEST) {
        PX4_INFO("already in test mode\n");
        // return 0;
      } else {
        PX4_INFO("entering test mode\n");
        mode = MODE_TEST;
        control[4] = 1.0;
        // return 0;
      }
    }

    else {
      PX4_INFO("usage: iq switch {flight|roll|test}\n");
      return -1;
    }

    control[0] = 0.0;
    control[1] = 0.0;
    control[2] = 0.0;
    control[3] = 0.0;
    return 0;
  }

  if (!strcmp(argv[1], "control")) {
    if (argc < 3) {
      PX4_INFO("usage: iq control {speed|voltage}\n");
      return -1;

    } else if (!thread_running) {
      PX4_INFO("Thread not running\n");
      return -1;

    } else if (is_armed) {
      PX4_INFO("Cannot change while armed\n");
      return -1;

    } else if (!strcmp(argv[2], "speed")) {
      if (iq_control_mode == IQ_CONTROL_SPEED) {
        PX4_INFO("already controlling speed in closed loop\n");
      } else {
        iq_control_mode = IQ_CONTROL_SPEED;
        PX4_INFO("now controlling speed\n");
        iq_reboot(TIMEOUT, SLEEP_TIME);
      }

    } else if (!strcmp(argv[2], "voltage")) {
      if (iq_control_mode == IQ_CONTROL_VOLTS) {
        PX4_INFO("already controlling voltage in closed loop\n");
      } else {
        iq_control_mode = IQ_CONTROL_VOLTS;
        PX4_INFO("now controlling voltage\n");
        iq_reboot(TIMEOUT, SLEEP_TIME);
      }
    }

    else {
      PX4_INFO("usage: iq control {speed|voltage}\n");
      return -1;
    }

    return 0;
  }

  if (!strcmp(argv[1], "actuator")) {
    if (argc < 3) {
      PX4_INFO("usage: iq actuator {pid|manual|spinner}\n");
      return -1;

    } else if (!strcmp(argv[2], "manual")) {
      if (actuator_control_id == ACTUATOR_MANUAL) {
        PX4_INFO("already using manual actuator controls\n");
      } else {
        PX4_INFO("actuator controls changing to manual\n");
        actuator_control_id = ACTUATOR_MANUAL;
      }

    } else if (!strcmp(argv[2], "pid")) {
      if (actuator_control_id == ACTUATOR_PID) {
        PX4_INFO("already using PID actuator controls\n");
      } else {
        PX4_INFO("actuator controls changing to PID\n");
        actuator_control_id = ACTUATOR_PID;
      }

    } else if (!strcmp(argv[2], "spinner")) {
      if (actuator_control_id == ACTUATOR_SPINNER) {
        PX4_INFO("already using SPINNER actuator controls\n");
      } else {
        PX4_INFO("actuator controls changing to SPINNER\n");
        actuator_control_id = ACTUATOR_SPINNER;
      }

    } else {
      PX4_INFO("usage: iq actuator {manual|pid|spinner}\n");
      return -1;
    }

    return 0;
  }

  if (!strcmp(argv[1], "param")) {
    if (argc < 3) {
      PX4_INFO("usage: iq param {load|print}\n");
      return -1;

    } else if (!strcmp(argv[2], "load")) {
      parameter_load_all(parameters, N_PARAMETERS, true);

    } else if (!strcmp(argv[2], "print")) {
      parameter_print_all(parameters, N_PARAMETERS);

    } else {
      PX4_INFO("usage: iq param {load|print}\n");
      return -1;
    }
    return 0;
  }

  if (!strcmp(argv[1], "power")) {
    if (argc > 3)
    {
      PX4_INFO("usage: iq power [exponent]");
      return -1;
    }

    double new_exponent = atof(argv[2]);
    if (new_exponent < 0.3 or new_exponent > 5){
      PX4_INFO("exponent must be between 0.3 and 5");
      return -1;
    }

    PX4_INFO("Exponent changing from %.2f to %.2f", voltage_exponent, new_exponent);
    voltage_exponent = new_exponent;
    return 0;
  }

  if (!strcmp(argv[1], "stop")) {
    if(is_armed)
    {
      PX4_ERR("cannot stop while armed");
      return -1;
    } else if (!thread_running) {
        PX4_INFO("already stopped\n");
        /* this is not an error */
        return 0;
    } else {
        PX4_INFO("stopping");
        thread_should_exit = true;
        return 0;
    }
  }

  if (!strcmp(argv[1], "status")) {
    PX4_INFO("voltage exponent: %.2f", voltage_exponent);

    switch(actuator_control_id){
        case ACTUATOR_MANUAL:
            PX4_INFO("actuator controls: manual");
            break;
        case ACTUATOR_PID:
            PX4_INFO("actuator controls: pid");
            break;
        case ACTUATOR_SPINNER:
            PX4_INFO("actuator controls: spinner");
            break;
    }

    switch(iq_control_mode){
        case IQ_CONTROL_SPEED:
            PX4_INFO("closed loop controller: SPEED");
            break;
        case IQ_CONTROL_VOLTS:
            PX4_INFO("closed loop controller: VOLTAGE");
            break;
    }

    if (thread_running) {
        switch(mode){
            case MODE_FLIGHT:
                PX4_INFO("running in flight mode");
                break;
            case MODE_ROLL:
                PX4_INFO("running in roll mode");
                break;
        }

    } else {
      PX4_INFO("stopped");
    }

    return 0;
  }

  PX4_ERR("unrecognized command");
  return 1;
}

/**
 * Print correct usage.
 */
static void usage(const char *reason) {
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}
	fprintf(stderr, "usage: iq {param|start|stop|status|switch|actuator|control|power}\n\n");
}
