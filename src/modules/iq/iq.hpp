#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <parameters/param.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
// #include <math.h>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#define clrscr() PX4_INFO("\e[1;1H\e[2J")

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/motor_temp.h>
#include <uORB/topics/iq_motors_state.h>

#include "comm/generic_interface.hpp"
#include "comm/serial_interface.hpp"
#include "comm/voltage_super_position_client.hpp"
#include "comm/system_control_client.hpp"
#include "comm/propeller_motor_control_client.hpp"
#include "comm/temperature_monitor_uc_client.hpp"
#include "comm/temperature_estimator_client.hpp"
#include "iq_parameters_helpers.hpp"
// #include "iq_helpers.cpp"

#define MODE_FLIGHT 0
#define MODE_ROLL 1
#define MODE_TEST 2
#define MODE_CALIBRATION 3
#define ACTUATOR_PID 0
#define ACTUATOR_MANUAL 3
#define ACTUATOR_SPINNER 5
#define ACTUATOR_IQTEST 7
#define IQ_CONTROL_SPEED 10
#define IQ_CONTROL_VOLTS 11
#define TIMEOUT 0.003
#define SLEEP_TIME 30000
#define TOPICS_TIME 150

/* Prototypes */
#define float_eq(a,b) (abs(a - b) < 0.001) ? true:false)
extern "C" __EXPORT int iq_main(int argc, char *argv[]);
static volatile bool thread_should_exit = false;   /**< Daemon exit flag */
static volatile bool thread_running = false;   /**< Daemon status flag */
static volatile int daemon_task;       /**< Handle of daemon task / thread */

static void usage(const char *reason);
int iq_thread_main(int argc, char *argv[]);
int iq_thread_temp(int argc, char *argv[]);
void enter_coast_mode();
void get_temperature();
void set_commands(float *actuator_control);
void send_commands(float *actuator_control);
int iq_reboot(float timeout, int sleep_time);
int init_system(float timeout, int sleep_time);
// int parameter_load_all();
// int parameter_print_all();
// int parameter_print_all(FILE* fptr);

bool is_coast = true;
bool is_armed = false;
bool was_armed = false;
bool state_log = false;
bool temp_is_dangerous = false;
bool temp_was_dangerous = false;
bool test_started = false;
short int mode = MODE_FLIGHT;
short int iq_control_mode = IQ_CONTROL_VOLTS;

// other variables
int actuator_control_id = ACTUATOR_MANUAL;
int error_counter = 0;
double thrust = 0.0f;
double x_roll = 0.0f, y_pitch = 0.0f, z_yaw = 0.0f;//, roll_speed = 0, volts = 0;
double amplitude = 0.0f, phase = 0.0f;
double voltage_exponent = 1.0f;
double q[4];
double gyro[3];
double acc[3];
float control[5];// = {0, 0, 0, 0, 0};
double thrust_diff = 0.0f;


// IQ communication variables
SerialInterface *com;
char *uart_name1;
PropellerMotorControlClient pmc1(0);
PropellerMotorControlClient pmc2(1);
VoltageSuperPositionClient vsc1(0);
VoltageSuperPositionClient vsc2(1);
TemperatureMonitorUcClient tuc1(0);
TemperatureMonitorUcClient tuc2(1);
TemperatureEstimatorClient tec1(0);
TemperatureEstimatorClient tec2(1);
SystemControlClient sys(0);


// uORB communication variables
struct orb_topic {
    int fd;
    const orb_metadata id;
    unsigned int interval;
};

int actuator_arm_sub_fd;
int actuator_ctrl_manual_sub_fd;
int actuator_ctrl_spinner_sub_fd;
int actuator_ctrl_iqtest_sub_fd;
int actuator_ctrl_sub_fd;
// int motor_temp_sub_fd;
// int actuator_ctrl_sub_fd;
// int sensor_combined_sub_fd;
// int vehicle_attitude_sub_fd;
struct motor_temp_s motor_temp_raw;
struct iq_motors_state_s iq_motors_state_raw;
orb_advert_t temp_pub;
orb_advert_t iq_motors_state_pub;
px4_pollfd_struct_t fds[5];

