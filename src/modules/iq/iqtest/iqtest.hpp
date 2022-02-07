#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
// #include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/iq_motors_state.h>
// #include <uORB/topics/wind_estimate.h>
// #include <uORB/topics/parameter_update.h>
// #include <uORB/topics/vehicle_global_position.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
// #include <lib/ecl/geo/geo.h>
// #include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <time.h>

#include "../iq_parameters_helpers.hpp"
// #include "test_phase.cpp"
// #include "test_speed.cpp"
// #include "test_speedpulse.cpp"
// #include "test_random.cpp"

// function prototypes
// double get_rand(double value_min, double value_max);
// void test_finish();
// int test_startup(char* test_type);
// void write_test_line();
// int publish_actuators();
#define float_eq(a,b) ((abs(a - b) < 0.001) ? true:false)
extern "C" __EXPORT int iqtest_main(int argc, char *argv[]);
static volatile int daemon_task;       /**< Handle of daemon task / thread */
FILE *fptr;

static void usage(const char *reason);
void enter_coast_mode();
int publish_actuators();

struct actuator_controls_s _actuators;
orb_advert_t _actuator_pub;
bool thread_running = false;
bool thread_should_exit = false;
bool thread_can_exit = false;

// uORB variables
static int sensor_combined_sub_fd;
static int vehicle_attitude_sub_fd;
static int vehicle_angular_velocity_sub_fd;
static int iq_motors_state_sub_fd;
static px4_pollfd_struct_t fds[4];

// test time
static int start, stop;
static int i, j;
static int plot_i;
int plot_start;
int plot_step = 5;
bool zero_pulse;
char test_filename[40] = "test";

// test inputs
double stop_signal_time = 2.0;

// speed options
double speed_min;
double speed_max;
int speed_n;

// pulse options
double pulse_min ;
double pulse_max;
int pulse_n;

// phase options
double phase_min;
double phase_max;
int phase_n;
bool phase_alternate;

// time options
double step_time;
double step_min;
double step_max;

// other
int random_n;
bool time_pulse_scale;
bool centering = 0;
bool should_go_backwards;
bool print_console = true;

double speed_list[50];
double pulse_list[50];
// float

// iq motor commands
static double pulse, speed, phase;

// float phase_test_time = 0.0;
// float q[4], acc[4], gyro[4];
