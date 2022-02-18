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
 * Helper functions for spinner.cpp
 *
 * @author Evandro Bernardes <evandro.bernardes@univ-amu.fr>
 */

#include "spinner.hpp"

/**
 ***************************************
 * SYSTEM INITIALIZATION HELPER FUNCTIONS
 ***************************************
 *
 */

/**
 * Load parameters, initialize topics and set UART communication to motors
 */
int init_system(){

  /*
	 * Safely initialize all structs to zero.
	 *
	 * These structs contain the system state and things
	 * like attitude, position, the current waypoint, etc.
	 */
	memset(&actuator_manual, 0, sizeof(actuator_manual));
	memset(&actuator_control, 0, sizeof(actuator_control));
	memset(&att, 0, sizeof(att));
	memset(&ang_vel, 0, sizeof(ang_vel));
	memset(&visual_odom, 0, sizeof(visual_odom));

  /* publish actuator controls with zero values */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuator_manual.control[i] = 0.0f;
		actuator_control.control[i] = 0.0f;
	}
  actuator_control.control[4] = 1.0f;

  /* subscribe to topics. */
  att_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
  visual_odom_sub_fd = orb_subscribe(ORB_ID(vehicle_visual_odometry));
  att_sp_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude_setpoint)); // manual control
  ang_vel_sub_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));
  ctrl_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3)); // manual control

  orb_set_interval(att_sub_fd, 1);
  orb_set_interval(visual_odom_sub_fd, 1);
  orb_set_interval(att_sp_sub_fd, 1);
  orb_set_interval(ang_vel_sub_fd, 1);
  orb_set_interval(ctrl_sub_fd, 1);

  fds[0] = (px4_pollfd_struct_t) { .fd = att_sub_fd, .events = POLLIN };
  fds[1] = (px4_pollfd_struct_t) { .fd = visual_odom_sub_fd, .events = POLLIN };
  fds[2] = (px4_pollfd_struct_t) { .fd = att_sp_sub_fd, .events = POLLIN };
  fds[3] = (px4_pollfd_struct_t) { .fd = ang_vel_sub_fd, .events = POLLIN };
  fds[4] = (px4_pollfd_struct_t) { .fd = ctrl_sub_fd, .events = POLLIN };

	uORB::Subscription parameter_update_sub{ORB_ID(parameter_update)};

  /* Setup of loop */
  return parameter_load_all(params, N_PARAMETERS, true);
  Binv = Id * params[PROP_K_RATIO].value + e.hat() * params[PROP_HEIGHT].value;
  Binv.I() = Binv.I();
}

/**
 ***************************************
 * uORB METHODS
 ***************************************
 *
 */
static int publish_actuators()
{
	actuator_control.timestamp = hrt_absolute_time();

	// lazily publish _actuators only once available
	if (actuator_pub != nullptr) {
		return orb_publish(ORB_ID(actuator_controls_spinner), actuator_pub, &actuator_control);

	} else {
		actuator_pub = orb_advertise(ORB_ID(actuator_controls_spinner), &actuator_control);

		if (actuator_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

static int read_topics(){
  // struct actuator_controls_s actuator_raw;

  /* get a local copy of attitude */
  orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub_fd, &att_sp);
  orb_copy(ORB_ID(vehicle_angular_velocity), ang_vel_sub_fd, &ang_vel);
  orb_copy(ORB_ID(actuator_controls_3), ctrl_sub_fd, &actuator_manual);

  qd = matrix::Quatf(att_sp.q_d);

  // get vehicle orientation
  switch(quat_mode) {
    case QUAT_EKF:
      orb_copy(ORB_ID(vehicle_attitude), att_sub_fd, &att);
      q = matrix::Quatf(att.q);
    break;
    case QUAT_VISUAL:
      orb_copy(ORB_ID(vehicle_visual_odometry), visual_odom_sub_fd, &visual_odom);
      q = matrix::Quatf(visual_odom.q);
    break;
    case QUAT_NULL:
      q = matrix::Quatf(1, 0, 0, 0);
    break;
  }

  // pass manual thrust command
  actuator_control.control[3] = actuator_manual.control[3];

  //
  rotor_velocity = (double) actuator_manual.control[3] * params[PROP_MAX_SPEED].value;

  // get goal mode
  switch(goal_mode) {
    case GOAL_MANUAL:
    {
      double x_roll    = actuator_manual.control[0];
      double y_pitch   = actuator_manual.control[1];
      double phase = atan2(y_pitch, x_roll);
      double inclination = fmin(1.0,sqrt(x_roll * x_roll + y_pitch * y_pitch));
      inclination = inclination*params[MAX_INCLINATION].value;

      // goal vector
      vd(0) = cos(phase)*sin(inclination);
      vd(1) = sin(phase)*sin(inclination);
      vd(2) = cos(inclination);
    }
    break;

    case GOAL_SETPOINT:
      vd = qd.conjugate(e);
      break;
  }

  // vehicle angular velocity
  w(0) = ang_vel.xyz[0];
  w(1) = ang_vel.xyz[1];
  w(2) = ang_vel.xyz[2];
  wd = -e * (rotor_velocity * params[ANG_VEL_RATIO].value);

  // normal vector
  v = q.conjugate(e);
  vdot = q.conjugate(w.cross(e));

  // mean normal vector
  if ((v+e).norm() > 0.001){
    p = (v+e);
  } else {
    p = p - e*(p.dot(e)); // avoid singularity by choosing the closest
  }
  p.normalize();

  vd.normalize();
  pd = (vd+e);
  pd.normalize();


}

/**
 ***************************************
 * CONTROL HELPER FUNCTIONS
 ***************************************
 *
 */

void control_attitude(){

  tau = get_torque();

  matrix::Vector3f tau_(tau);
  tau_.normalize();

  actuator_control.control[0] = tau_(0);
  actuator_control.control[1] = tau_(1);
  actuator_control.control[2] = 0;
  actuator_control.control[3] = actuator_manual.control[3];

}

matrix::Vector<float, 3> get_torque(){

  matrix::Vector3f tau_p = q.conjugate_inversed(p.cross(pd) / p.dot(pd));

  matrix::Vector3f wb = e * (p.dot(w) / p.dot(e));
  //matrix::Vector3f tau_d = (w - wb) * params[SPIN_KD].value;

  matrix::Vector3f tau_d = vdot - p.dot(vdot)*p;
  tau_d = tau_d/(v+e).norm();


  matrix::Vector3f tau_thrust;
  if (thrust_mode == THRUST_SPIN){
    tau_thrust = (wd - wb) * params[SPIN_KS].value;
  } else if (thrust_mode == THRUST_CONSTANT) {
    tau_thrust = e * rotor_velocity * rotor_velocity * params[SPIN_KC].value * params[PROP_K_RATIO].value;
  }

  return tau_p * params[SPIN_KP].value - tau_d * params[SPIN_KD].value + tau_thrust;
}

/**
 ***************************************
 * ROTATION DECOMPOSITION FUNCTIONS
 ***************************************
 *
 */

float wrapTo180(float angle_deg){
  float angle_new = fmod(angle_deg + 180, 360);
  if (angle_new < 0)
    angle_new += 360;
  return angle_new - 180;
}

