#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <stdio.h>
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
#include <uORB/topics/angles.h>
#include <uORB/topics/sensor_combined.h>
// #include <uORB/topics/wind_estimate.h>
// #include <uORB/topics/parameter_update.h>
// #include <uORB/topics/vehicle_global_position.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
// #include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <time.h>

extern "C" __EXPORT int get_angles_main(int argc, char *argv[]);
static volatile int daemon_task;       /**< Handle of daemon task / thread */

static void usage(const char *reason);
bool attitude_thread_running = false;

float get_norm(float *array, int length);
void normalize(float *array, int length);

int get_angles_thread_main(int argc, char *argv[])
{

    // set uORB topic subscribers
    int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    int actuator_ctrl_manual_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3)); // manual control
    int sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    orb_set_interval(sensor_combined_sub_fd, 1);
    orb_set_interval(vehicle_attitude_sub_fd, 1);
    orb_set_interval(actuator_ctrl_manual_sub_fd, 1);
    px4_pollfd_struct_t fds[] = {
        {.fd = vehicle_attitude_sub_fd,   .events = POLLIN },
        {.fd = actuator_ctrl_manual_sub_fd,   .events = POLLIN },
        {.fd = sensor_combined_sub_fd,   .events = POLLIN },};

    /* advertise angles topic */
	struct angles_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(angles), &att);


    attitude_thread_running = true;
    int error_counter = 0;

    while(attitude_thread_running){
        // handle errors
		int poll_ret = px4_poll(fds, 1, 150);
        if (poll_ret > 0) {
            struct actuator_controls_s actuator_raw;
            orb_copy(ORB_ID(actuator_controls_3), actuator_ctrl_manual_sub_fd, &actuator_raw);
            att.manual[0]   = actuator_raw.control[0];
            att.manual[1]   = actuator_raw.control[1];
            att.manual[2]   = actuator_raw.control[2];
            att.phase_manual = atan2(att.manual[1],att.manual[0])*M_RAD_TO_DEG_F;

            struct vehicle_attitude_s vehicle_attitude_raw;
            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &vehicle_attitude_raw);
            float qa_0 = vehicle_attitude_raw.q[0];
            float qx_0 = vehicle_attitude_raw.q[1];
            float qy_0 = vehicle_attitude_raw.q[2];
            float qz_0 = vehicle_attitude_raw.q[3];

            // Converting from NED to ENU:
            // float qa = -(qx_0-qy_0)/sqrt(2);
            // float qx = +(qa_0-qz_0)/sqrt(2);
            // float qy = -(qa_0+qz_0)/sqrt(2);
            // float qz = +(qx_0+qy_0)/sqrt(2);
            float qa = qa_0;
            float qx = qx_0;
            float qy = qy_0;
            float qz = qz_0;

            float ez1 = 2*(qx*qz + qa*qy);
            float ez2 = 2*(qy*qz - qa*qx);
            float ez3 = 1 - 2*(qx*qx + qy*qy);

            att.up[0] = -ez2;
            att.up[1] = ez1;
            att.up[2] = -ez3;

            // ZYZ angles from quaternion
            att.zyz[0] = atan2(ez2,-ez1);
            att.zyz[1] = acos(ez3);
            // att.zyz[2] = atan2(ez2,+ez1);
            att.zyz[2] = atan2(+(qy*qz + qa*qz),+(qx*qz - qa*qy));
            att.zyz_deg[0] = (att.zyz[0]*M_RAD_TO_DEG_F);
            att.zyz_deg[1] = (att.zyz[1]*M_RAD_TO_DEG_F);
            att.zyz_deg[2] = (att.zyz[2]*M_RAD_TO_DEG_F);

            // XYZ angles from quaternion
            att.xyz[0] = atan2(2*(qa*qx + qy*qz),1 - 2*(qx*qx + qy*qy));
            att.xyz[1] = asin(2*(qa*qy - qx*qz));
            att.xyz[2] = atan2(2*(qa*qz + qx*qy),1 - 2*(qy*qy + qz*qz));
            att.xyz_deg[0] = (att.xyz[0]*M_RAD_TO_DEG_F);
            att.xyz_deg[1] = (att.xyz[1]*M_RAD_TO_DEG_F);
            att.xyz_deg[2] = (att.xyz[2]*M_RAD_TO_DEG_F);

            att.phase_diff = att.zyz_deg[0] - att.phase_manual;

            struct sensor_combined_s sensor_combined_raw;
            orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &sensor_combined_raw);
            att.acc[0] = sensor_combined_raw.accelerometer_m_s2[0];
            att.acc[1] = sensor_combined_raw.accelerometer_m_s2[1];
            att.acc[2] = sensor_combined_raw.accelerometer_m_s2[2];
            normalize(att.acc,3);

            orb_publish(ORB_ID(angles), att_pub, &att);
            //PX4_INFO("phase: %.2f\tphase2: %.2f\tinc: %.2f\n",phase*M_RAD_TO_DEG_F,phase2*M_RAD_TO_DEG_F,inclination*M_RAD_TO_DEG_F);
        }
    }
}

int get_angles_main(int argc, char *argv[]){

    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (attitude_thread_running) {
            PX4_INFO("already running\n");
            /* this is not an error */
            return 0;
        }
        attitude_thread_running = true;

        daemon_task = px4_task_spawn_cmd("get_attitude",
                SCHED_DEFAULT,
                SCHED_PRIORITY_DEFAULT,
                2001,
                get_angles_thread_main,
                //(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
                //  (argv) ? (char *const *)&uart_name1 : (char *const *)NULL);
                (char *const *)NULL);

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (!attitude_thread_running) {
            PX4_INFO("already stopped\n");
            /* this is not an error */
        } else {
            PX4_INFO("stopping");
            attitude_thread_running = false;
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
	fprintf(stderr, "usage: get_attitude {start|stop|status}\n\n");
}

float get_norm(float *array, int length)
{
    float result = 0;
    for(int i = 0; i < length; i++){
        result += array[i]*array[i];
    }
    return sqrt(result);
}

void normalize(float *array, int length)
{
    float norm = get_norm(array, length);
    for(int i = 0; i < length; i++){
        array[i] /= norm;
    }
}

