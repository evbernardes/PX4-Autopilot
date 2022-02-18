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
 * @file spinner.hpp
 * Controller prototype for single rotor drone
 *
 * @author Evandro Bernardes <evandro.bernardes@univ-amu.fr>
 */
#ifndef SPINNER_H
#define SPINNER_H
#include "params.h"

#include <poll.h>

#include <drivers/drv_hrt.h>
// #include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
// #include <mavlink_shell.h>
#include <uORB/topics/vehicle_attitude_extra.h>
#include <uORB/topics/vehicle_odometry.h>
#include "spinner_parameters.hpp"

#define QUAT_ENU 0
#define QUAT_NED 1
#define QUAT_NULL -1
#define QUAT_EKF 5
#define QUAT_VISUAL 10
#define THRUST_SPIN 0
#define THRUST_CONSTANT 1
#define GOAL_MANUAL 0
#define GOAL_SETPOINT 1

/* Prototypes */
extern "C" int parameters_init(struct param_handles *h);
extern "C" int parameters_update(const struct param_handles *h, struct params *p);
extern "C" __EXPORT int spinner_main(int argc, char *argv[]);
extern int parameter_load(char *param_name, double *param_value);
static void usage(const char *reason);
static int spinner_thread_main(int argc, char *argv[]);
static int init_system();
static int publish_actuators();
static int read_topics();
static matrix::Vector<float, 3> get_torque();
static void control_attitude();
matrix::Quatf q_red_from_vector(matrix::Vector3f vector_up);
matrix::Vector3f v_red_from_quat(matrix::Quatf quat);

static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

static struct actuator_controls_s actuator_manual;
static struct actuator_controls_s actuator_control;
static struct vehicle_attitude_s att;
static struct vehicle_odometry_s visual_odom;
// static struct vehicle_attitude_s att_sp;
static struct vehicle_attitude_extra_s att_extra;
static struct vehicle_angular_velocity_s ang_vel;
static struct vehicle_attitude_setpoint_s att_sp;

static orb_advert_t actuator_pub;
static orb_advert_t att_extra_pub;
// orb_advert_t rates_pub;

static int att_sub_fd;
static int ang_vel_sub_fd;
static int ctrl_sub_fd;
static int att_sp_sub_fd;
static int visual_odom_sub_fd;

static px4_pollfd_struct_t fds[5];

matrix::Quaternionf Q_a(matrix::Vector3f v);
matrix::Quaternionf Q_a(matrix::Quaternionf q);
// matrix::SquareMatrix<float, 3> get_B_inv(matrix::Quaternionf q, matrix::Vector3f w);
float wrapTo180(float angle_deg);

static matrix::Vector3f tau(0.0f,0.0f,0.0f);
static matrix::Vector3f thrust(0.0f,0.0f,0.0f);
// static matrix::Vector3f v(0.0f,0.0f,1.0f);
static matrix::Vector3f e(0.0f,0.0f,1.0f);
static matrix::Vector3f v_null(1.0f,0.0f,1.0f);
// e.normalize();
static matrix::Quatf eq(0.0, e(0), e(1), e(2));
static matrix::Quatf q(1,0,0,0);
static matrix::Quatf qd(1,0,0,0);

static matrix::Vector3f w(e*0);
static matrix::Vector3f v(e);
static matrix::Vector3f p(e);
static matrix::Vector3f wd(0, 0, 0);
static matrix::Vector3f vd(e);
static matrix::Vector3f pd(e);

float Id_data[3*3] = {
  1.0, 0.0, 0.0,
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0};
static matrix::SquareMatrix<float, 3> Id(Id_data);

float B_inv_data[3*3] = {
  +0.5, 0.5, 0.0,
  -0.5, 0.5, 0.0,
  +0.0, 0.0, 1.0};
static matrix::SquareMatrix<float, 3>

Binv(B_inv_data);
// static matrix::SquareMatrix<float, 3> B_;
// static matrix::SquareMatrix<float, 3> J_;

short int quat_mode = QUAT_NED;
short int quat_source = QUAT_EKF;
short int thrust_mode = THRUST_SPIN;
short int goal_mode = GOAL_SETPOINT;
double rotor_velocity = 0.0f;
static int time_0 = 0.0f;
static int time_1 = 0.0f;
float pot_energy = 0;
bool is_debug = false;
// bool s = false;

#endif /* SPINNER_H */

