#include "iqtest.hpp"

// function prototypes
double get_rand(double value_min, double value_max);
void test_finish();
int test_startup(char* test_type);
void write_test_line();
int publish_actuators();

// function thread starters and main
#include "test_phase.cpp"
#include "test_speed.cpp"
#include "test_speedpulse.cpp"
#include "test_random.cpp"
#include "test_free.cpp"

int iqtest_main(int argc, char *argv[]){

    if (argc < 2) {
      usage("missing command");
      return 1;
    }

    if (!strcmp(argv[1], "stop")) {
        if (!thread_running) {
            PX4_INFO("already stopped\n");
            /* this is not an error */
        } else {
            PX4_INFO("stopping");
            thread_should_exit = true;
        }
        return 0;
    }

    if (!strcmp(argv[1], "random")){
      if (thread_running) {
        PX4_INFO("already running\n");
        return -1;
      }
      return iqtest_random_thread_starter(argc, argv);
    }

    if (!strcmp(argv[1], "speed")) {
      if (thread_running) {
        PX4_INFO("already running\n");
        return -1;
      }
      return iqtest_speed_thread_starter(argc, argv);
    }

    if (!strcmp(argv[1], "speedpulse")) {
      if (thread_running) {
        PX4_INFO("already running\n");
        return -1;
      }
      return iqtest_speedpulse_thread_starter(argc, argv);
    }

    if (!strcmp(argv[1], "phase")) {
      if (thread_running) {
        PX4_INFO("already running\n");
        return -1;
      }
      return iqtest_phase_thread_starter(argc, argv);
    }

    if (!strcmp(argv[1], "free")) {
      if (thread_running) {
        PX4_INFO("already running\n");
        return -1;
      }
      return iqtest_free_thread_starter(argc, argv);
    }

    if (!strcmp(argv[1], "printmode")) {
        print_console = (bool) atoi(argv[2]);
        if (print_console){
          PX4_INFO("Printing to console...\n");
        } else {
          PX4_INFO("NOT printing to console...\n");
        }
        return 0;
    }

    PX4_ERR("unrecognized command");
    return 1;
}

//
// HELPER FUNCTIONS
//

/**
 * Print correct usage.
 */
static void usage(const char *reason) {
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}
	fprintf(stderr, "usage: iqtest {speed|speedpulse|phase|free|random|stop|printmode}\n\n");
}

/**
 * give double random number from value_min to value_max
 */
double get_rand(double value_min, double value_max){
  return (double) (value_max - value_min)*rand()/RAND_MAX + value_min;
}

/**
 * create file, set uORB topics and load parameters
 */
int test_startup(char* test_type){
  char filepath[62];
  sprintf(filepath,"/fs/microsd/test/%s.dat",test_filename);
  fptr = fopen(filepath, "w");
  fprintf(fptr, "TEST_TYPE = %s\n",test_type);
  // set uORB topic subscribers
  sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
  vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
  vehicle_angular_velocity_sub_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));
  iq_motors_state_sub_fd = orb_subscribe(ORB_ID(iq_motors_state));
  orb_set_interval(sensor_combined_sub_fd, 1);
  orb_set_interval(vehicle_attitude_sub_fd, 1);
  orb_set_interval(vehicle_angular_velocity_sub_fd, 1);
  orb_set_interval(iq_motors_state_sub_fd, 1);
  fds[0] = (px4_pollfd_struct_t) { .fd = sensor_combined_sub_fd,   .events = POLLIN };
  fds[1] = (px4_pollfd_struct_t) { .fd = vehicle_attitude_sub_fd,   .events = POLLIN };
  fds[2] = (px4_pollfd_struct_t) { .fd = vehicle_angular_velocity_sub_fd,   .events = POLLIN };
  fds[3] = (px4_pollfd_struct_t) { .fd = iq_motors_state_sub_fd,   .events = POLLIN };

  thread_running = true;
  thread_should_exit = false;
//   int error_counter = 0;

  _actuators.control[0] = 0.0f;
  _actuators.control[1] = 0.0f;
  _actuators.control[2] = 0.0f;
  _actuators.control[3] = 0.0f;
  _actuators.control[4] = 1.0f;

  parameter_load_all(parameters, N_PARAMETERS, false);
  parameter_print_all(parameters, N_PARAMETERS, fptr);
  fprintf(fptr, "rotor_speed_limits = [%.2f,%.2f], %d steps\n",speed_min * parameters[PROP_MAX_SPEED].value,speed_max * parameters[PROP_MAX_SPEED].value,speed_n);
  fprintf(fptr, "pulse = [%.2f,%.2f], %d steps\n",pulse_min * parameters[PROP_MAX_PULSE].value,pulse_max * parameters[PROP_MAX_PULSE].value,pulse_n);
  fprintf(fptr, "pulse_phase = [%.2f,%.2f]\n", phase_min, phase_max);
  fprintf(fptr, "step_time = %.2fs\n",step_time);
  fprintf(fptr, "slot_pause_time = %d seconds \n", plot_step);
  fprintf(fptr, "* format: "
                "time \t i \t rotor_speed \t pulse \t pulse_phase \t "
                "qa \t qx \t qy \t qz \t "
                "gyro_x \t gyro_y \t gyro_z \t "
                "acc_x \t acc_y \t acc_z \t "
                "angvel_x \t angvel_y \t angvel_z \t "
                "iq_vel_up \t iq_vel_down \t iq_volts_up \t iq_volts_down \t "
                "iq_pulse_up \t iq_pulse_down \t iq_phase_up \t iq_phase_down \t");
  fprintf(fptr, "***\n");

  start = hrt_absolute_time();
  plot_start = start;
  stop = hrt_absolute_time();
  PX4_INFO("Starting test...");
  PX4_INFO("Getting starting data, time = %.2f",start/1000000.0);
  return 0;
}

/**
 * close file and stop test
 */
void test_finish(){
  phase = 0.0;
  pulse = 0.0;
  speed = 0.0;
  // enter_coast_mode();
  start = hrt_absolute_time();
  stop = hrt_absolute_time();
  PX4_INFO("Sending stop signal..., time = %.2f",stop/1000000.0);
  thread_should_exit = false;
  while(!thread_should_exit){
      if((stop - start)/1000000.0 > stop_signal_time)
          thread_should_exit = true;
      stop = hrt_absolute_time();
      publish_actuators();
  }
  PX4_INFO("Stop signal sent, time = %.2f",stop/1000000.0);
  PX4_INFO("Test finished, exiting.");
  fclose(fptr);
  thread_running = false;
}


/**
 * write one line of data
 */
void write_test_line(){
  if (thread_should_exit) return;

  int poll_ret = px4_poll(fds, 2, 150);
  if (poll_ret > 0) {
    if ((stop - plot_start)/1000 > plot_step){

      struct vehicle_attitude_s vehicle_attitude_raw;
      orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &vehicle_attitude_raw);
      struct sensor_combined_s sensor_combined_raw;
      orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &sensor_combined_raw);
      struct vehicle_angular_velocity_s vehicle_angular_velocity_raw;
      orb_copy(ORB_ID(vehicle_angular_velocity), vehicle_angular_velocity_sub_fd, &vehicle_angular_velocity_raw);
      struct iq_motors_state_s iq_motors_state_raw;
      orb_copy(ORB_ID(iq_motors_state), iq_motors_state_sub_fd, &iq_motors_state_raw);

      fprintf(fptr, "%d \t %d \t %f \t %f \t %f \t "
                    "%f \t %f \t %f \t %f \t "
                    "%f \t %f \t %f \t "
                    "%f \t %f \t %f \t "
                    "%f \t %f \t %f \t "
                    "%f \t %f \t %f \t %f \t "
                    "%f \t %f \t %f \t %f \n",
              stop,
              plot_i,
              (double)speed*parameters[PROP_MAX_SPEED].value,
              (double)pulse*parameters[PROP_MAX_PULSE].value,
              (double)phase,
              (double)vehicle_attitude_raw.q[0],
              (double)vehicle_attitude_raw.q[1],
              (double)vehicle_attitude_raw.q[2],
              (double)vehicle_attitude_raw.q[3],
              (double)sensor_combined_raw.gyro_rad[0],
              (double)sensor_combined_raw.gyro_rad[1],
              (double)sensor_combined_raw.gyro_rad[2],
              (double)sensor_combined_raw.accelerometer_m_s2[0],
              (double)sensor_combined_raw.accelerometer_m_s2[1],
              (double)sensor_combined_raw.accelerometer_m_s2[2],
              (double)vehicle_angular_velocity_raw.xyz[0],
              (double)vehicle_angular_velocity_raw.xyz[1],
              (double)vehicle_angular_velocity_raw.xyz[2],
              (double)iq_motors_state_raw.mean_velocity[0],
              (double)iq_motors_state_raw.mean_velocity[1],
              (double)iq_motors_state_raw.mean_volts[0],
              (double)iq_motors_state_raw.mean_volts[1],
              (double)iq_motors_state_raw.pulse_volts[0],
              (double)iq_motors_state_raw.pulse_volts[1],
              (double)iq_motors_state_raw.pulse_phase[0],
              (double)iq_motors_state_raw.pulse_phase[1]);
      plot_start = hrt_absolute_time();
    }
  }
}

/**
 * send commands
 */
int publish_actuators()
{
  _actuators.control[0] = pulse * cos(phase);
  _actuators.control[1] = pulse * sin(phase);
  _actuators.control[3] = speed;
	_actuators.timestamp = hrt_absolute_time();

	// lazily publish _actuators only once available
	if (_actuator_pub != nullptr) {
		return orb_publish(ORB_ID(actuator_controls_iqtest), _actuator_pub, &_actuators);

	} else {
		_actuator_pub = orb_advertise(ORB_ID(actuator_controls_iqtest), &_actuators);

		if (_actuator_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}
