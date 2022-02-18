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

static int publish_att_extra()
{
	actuator_control.timestamp = hrt_absolute_time();

	// lazily publish _actuators only once available
	if (att_extra_pub != nullptr) {
		return orb_publish(ORB_ID(spinneratt), att_extra_pub, &att_extra);

	} else {
		att_extra_pub = orb_advertise(ORB_ID(spinneratt), &att_extra);

		if (att_extra_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

static int read_topics(){
  bool is_from_manual;
  // struct actuator_controls_s actuator_raw;

  /* get a local copy of attitude */
  orb_copy(ORB_ID(vehicle_attitude), att_sub_fd, &att);
  orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub_fd, &att_sp);
  orb_copy(ORB_ID(vehicle_angular_velocity), ang_vel_sub_fd, &ang_vel);
  orb_copy(ORB_ID(actuator_controls_3), ctrl_sub_fd, &actuator_manual);
  orb_copy(ORB_ID(vehicle_visual_odometry), visual_odom_sub_fd, &visual_odom);



  // orb_copy(ORB_ID(actuator_controls_3), ctrl_sub_fd, &actuator_raw);

  if (quat_mode == QUAT_NED){
    q = matrix::Quatf(att.q);
    qd = matrix::Quatf(att_sp.q_d);
  } else if (quat_mode == QUAT_ENU){
    q = matrix::Quatf(att.q[0],att.q[2],att.q[1],-att.q[3]);
    qd = matrix::Quatf(att_sp.q_d[0],att_sp.q_d[2],att_sp.q_d[1],-att_sp.q_d[3]);
  } else if (quat_mode == QUAT_NULL){
    q = matrix::Quatf(1, 0, 0, 0);
    qd = matrix::Quatf(1, 0, 0, 0);
  }

  if (is_debug){
    static matrix::Vector3f p_null = (v_null.normalized() + e);
    float dot = q(1)*e(0) + q(2) + e(1) + q(3)*e(2);
    matrix::Quatf qb(dot,-q(0)*e(0),-q(0)*e(1),-q(0)*e(2));
    qb.normalize();
    q = matrix::Quatf(0, p_null(0), p_null(1), p_null(2)) * qb;
    q.normalize();
  }

  if (goal_mode == GOAL_MANUAL){
    // if (abs(actuator_raw.control[4] - actuator_control.control[4]) < 0.001){
      // is_from_manual = false;
    // } else {
    // memcpy(actuator_manual.control, actuator_raw.control, sizeof actuator_raw.control); // updates if control[4] matches
    is_from_manual = true;

    actuator_control.control[3] = actuator_manual.control[3]; // pass manual thrust command
    rotor_velocity = (double) actuator_manual.control[3] * params[PROP_MAX_SPEED].value;
    double x_roll    = actuator_manual.control[0];
    double y_pitch   = actuator_manual.control[1];
    double phase = atan2(y_pitch,x_roll);
    double inclination = fmin(1.0,sqrt(x_roll * x_roll + y_pitch * y_pitch));
    inclination = inclination*params[MAX_INCLINATION].value;

    // goal vector
    vd(0) = cos(phase)*sin(inclination);
    vd(1) = sin(phase)*sin(inclination);
    vd(2) = cos(inclination);
    // vd(0) = 0.0;
    // vd(1) = 0.0;
    // vd(2) = 1.0;

    // }
  } else if (goal_mode == GOAL_SETPOINT) {
    // memcpy(actuator_manual.control, actuator_raw.control, sizeof actuator_raw.control); // updates if control[4] matches
    actuator_control.control[3] = actuator_manual.control[3]; // pass manual thrust command
    rotor_velocity = (double)actuator_manual.control[3] * params[PROP_MAX_SPEED].value;
    vd = qd.conjugate(e);
  }

  v = q.conjugate(e);
  float N = (v+e).norm();
  if (N < 0.01){
    p = p - e*(p.dot(e));
    p.normalize();
  } else {
    p = (v+e)/N;
  }

  vd.normalize();
  pd = (vd+e);
  pd.normalize();

  // vehicle angular velocity
  if (quat_mode == QUAT_NED){
    w(0) = ang_vel.xyz[0];
    w(1) = ang_vel.xyz[1];
    w(2) = ang_vel.xyz[2];
    wd = -e * (rotor_velocity * params[ANG_VEL_RATIO].value);
  } else if (quat_mode == QUAT_ENU){
    w(0) = ang_vel.xyz[1];
    w(1) = ang_vel.xyz[0];
    w(2) = -ang_vel.xyz[2];
    wd = e * (rotor_velocity * params[ANG_VEL_RATIO].value);
  }

}

void set_att_message(){

  time_1 = hrt_absolute_time();
  att_extra.freq = 1000. / (time_1 - time_0);
  time_0 = time_1;

  att_extra.q[0] = q(0);
  att_extra.q[1] = q(1);
  att_extra.q[2] = q(2);
  att_extra.q[3] = q(3);

  att_extra.e[0] = e(0);
  att_extra.e[1] = e(1);
  att_extra.e[2] = e(2);

  att_extra.v[0] = v(0);
  att_extra.v[1] = v(1);
  att_extra.v[2] = v(2);

  att_extra.vd[0] = vd(0);
  att_extra.vd[1] = vd(1);
  att_extra.vd[2] = vd(2);

  att_extra.wd[0] = wd(0);
  att_extra.wd[1] = wd(1);
  att_extra.wd[2] = wd(2);

  att_extra.w[0] = w(0);
  att_extra.w[1] = w(1);
  att_extra.w[2] = w(2);

  att_extra.p[0] = p(0);
  att_extra.p[1] = p(1);
  att_extra.p[2] = p(2);

  matrix::Eulerf xyz(q);
  att_extra.euler_xyz[0] = xyz(0)*M_RAD_TO_DEG;
  att_extra.euler_xyz[1] = xyz(1)*M_RAD_TO_DEG;
  att_extra.euler_xyz[2] = xyz(2)*M_RAD_TO_DEG;


  att_extra.euler_zyz[0] = atan2(v(1),v(0))*M_RAD_TO_DEG;
  att_extra.euler_zyz[1] = acos(v(2))*M_RAD_TO_DEG;
  // att_extra.euler_zyz[2] = 0*M_RAD_TO_DEG_F;
  // matrix::EulerZYZf zyz(q);
  // att_extra.euler_zyz[0] = zyz(0)*M_RAD_TO_DEG_F;
  // att_extra.euler_zyz[1] = zyz(1)*M_RAD_TO_DEG_F;
  // att_extra.euler_zyz[2] = zyz(2)*M_RAD_TO_DEG_F;

  matrix::Quatf qb = q.extract_component_right(e);
  att_extra.qb[0] = qb(0);
  att_extra.qb[1] = qb(1);
  att_extra.qb[2] = qb(2);
  att_extra.qb[3] = qb(3);

  att_extra.theta = wrapTo180(2*atan2(e.dot(qb.imag()), qb(0))*M_RAD_TO_DEG_F);

  matrix::Quatf qa = q * qb.inversed();
  att_extra.qa[0] = qa(0);
  att_extra.qa[1] = qa(1);
  att_extra.qa[2] = qa(2);
  att_extra.qa[3] = qa(3);

  matrix::Vector3f C = pd.cross(p);
  att_extra.qdelta[0] = pd.dot(p);
  att_extra.qdelta[1] = C(0);
  att_extra.qdelta[2] = C(1);
  att_extra.qdelta[3] = C(2);

  att_extra.pot_energy = pot_energy;

  // matrix::Vector3f tau_p = q.conjugate_inversed(p.cross(pd) / p.dot(pd));
  matrix::Vector3f tau_p(tau);
  // att_extra.torque[0] = tau(0);
  // att_extra.torque[1] = tau(1);
  // att_extra.torque[2] = tau(2);
  att_extra.torque[0] = tau_p(0);
  att_extra.torque[1] = tau_p(1);
  att_extra.torque[2] = tau_p(2);

  matrix::Vector3f fp = Binv * tau_p;
  att_extra.thrust[0] = fp(0);
  att_extra.thrust[1] = fp(1);
  att_extra.thrust[2] = fp(2);

  fp.normalize();
  att_extra.n[0] = fp(0);
  att_extra.n[1] = fp(1);
  att_extra.n[2] = fp(2);

  fp = q.conjugate(fp);
  att_extra.ne[0] = fp(0);
  att_extra.ne[1] = fp(1);
  att_extra.ne[2] = fp(2);
  // att_extra.thrust[0] = (tau_p(1) + tau_p(0))/2;
  // att_extra.thrust[1] = (tau_p(1) - tau_p(0))/2;
  // att_extra.thrust[2] = tau_p(2);
  // att_extra.thrust[0] = (tau(1) + tau(0))/2;
  // att_extra.thrust[1] = (tau(1) - tau(0))/2;
  // att_extra.thrust[2] = tau(2);

  tau_p.normalize();
  att_extra.torque[0] = tau_p(0);
  att_extra.torque[1] = tau_p(1);
  att_extra.torque[2] = tau_p(2);

  att_extra.swash_deg[0] = atan2(att_extra.n[1], att_extra.n[0]) * M_RAD_TO_DEG;
  att_extra.swash_deg[1] = acos(att_extra.n[2]) * M_RAD_TO_DEG;

  att_extra.euler_zyz[2] = att_extra.euler_zyz[0] - atan2(att_extra.ne[1], att_extra.ne[0]);



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


  if (quat_mode == QUAT_NED){
    actuator_control.control[0] = tau_(0);
    actuator_control.control[1] = tau_(1);
  } else if (quat_mode == QUAT_ENU){
    actuator_control.control[0] = tau_(1);
    actuator_control.control[1] = tau_(0);
  }

  actuator_control.control[2] = 0;
  actuator_control.control[3] = actuator_manual.control[3];

  att_extra.control[0] = actuator_control.control[0];
  att_extra.control[1] = actuator_control.control[1];
  att_extra.control[2] = actuator_control.control[2];
  att_extra.control[3] = actuator_control.control[3];

}

matrix::Vector<float, 3> get_torque(){

  matrix::Vector3f tau_p = q.conjugate_inversed(p.cross(pd) / p.dot(pd));

  matrix::Vector3f wb = e * (p.dot(w) / p.dot(e));
  //matrix::Vector3f tau_d = (w - wb) * params[SPIN_KD].value;

  matrix::Vector3f v = q.conjugate(e);
  matrix::Vector3f vdot = q.conjugate(w.cross(e));
  matrix::Vector3f tau_d = vdot - p.dot(vdot)*p;
  tau_d = tau_d/(v+e).norm();


  matrix::Vector3f tau_thrust;
  if (thrust_mode == THRUST_SPIN){
    tau_thrust = (wd - wb) * params[SPIN_KS].value;
  } else if (thrust_mode == THRUST_CONSTANT) {
    tau_thrust = e * rotor_velocity * rotor_velocity * params[SPIN_KC].value * params[PROP_K_RATIO].value;
  }

  pot_energy = -(params[SPIN_KP].value/4)*log(p.dot(pd));
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

