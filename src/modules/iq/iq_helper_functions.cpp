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
 * @file iq_helpers.cpp
 * Definition of helper functions for communication with IQinetics motor
 *
 * @author Evandro Bernardes <evandro.bernardes@univ-amu.fr>
 */

/**
 ***************************************
 * SYSTEM INITIALIZATION HELPER FUNCTIONS
 ***************************************
 */
int init_system(float timeout, int sleep_time){
  // start UART communication interface with motors
  com = new SerialInterface(uart_name1);
  PX4_INFO("Opened %s with fd %d", uart_name1, com->get_fd());
  iq_reboot(timeout, sleep_time);

  control[0] = 0.0;
  control[1] = 0.0;
  control[2] = 0.0;
  control[3] = 0.0;
  control[4] = 0.0;

  // int send_ret = send_commands(control);
  // if(send_ret != 0) PX4_WARN("serial send error %d", send_ret);

  // set ORB topics system
  actuator_arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
  actuator_ctrl_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0)); // PID control
  actuator_ctrl_manual_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3)); // manual control
  actuator_ctrl_spinner_sub_fd = orb_subscribe(ORB_ID(actuator_controls_spinner)); // spinner control
  orb_set_interval(actuator_arm_sub_fd, 1);
  orb_set_interval(actuator_ctrl_sub_fd, 1);
  orb_set_interval(actuator_ctrl_manual_sub_fd, 1);
  orb_set_interval(actuator_ctrl_spinner_sub_fd, 1);
  fds[0] = (px4_pollfd_struct_t) { .fd = actuator_arm_sub_fd,   .events = POLLIN };
  fds[1] = (px4_pollfd_struct_t) { .fd = actuator_ctrl_sub_fd,   .events = POLLIN };
  fds[2] = (px4_pollfd_struct_t) { .fd = actuator_ctrl_manual_sub_fd,   .events = POLLIN };
  fds[3] = (px4_pollfd_struct_t) { .fd = actuator_ctrl_spinner_sub_fd,   .events = POLLIN };

  return parameter_load_all(parameters, N_PARAMETERS, true);
}

/**
 ***************************************
 * IQ MOTOR HELPER FUNCTIONS
 ***************************************
 */

void get_temperature()
{
  // get data asked on last iteration before asking for new data now
  // this way we're sure to leave enough time
  com->GetBytes();
  uint8_t *rx_data;// temporary pointer to received type+data bytes
  uint8_t rx_length; // number of received type+data bytes
  motor_temp_raw.n = 0;
  com->PeekPacket(&rx_data,&rx_length);
  // while we have message packets to parse
  while(com->PeekPacket(&rx_data,&rx_length))
  {
      // Share that packet with all client objects
      tec1.ReadMsg(*com,rx_data,rx_length);
      tec2.ReadMsg(*com,rx_data,rx_length);
      tuc1.ReadMsg(*com,rx_data,rx_length);
      tuc2.ReadMsg(*com,rx_data,rx_length);
      motor_temp_raw.n++;
      com->DropPacket(); // Once we’re done with the message packet, drop it
  }
  motor_temp_raw.tec[0] = tec1.temp_.get_reply();
  motor_temp_raw.tec[1] = tec2.temp_.get_reply();
  motor_temp_raw.tuc[0] = tuc1.uc_temp_.get_reply();
  motor_temp_raw.tuc[1] = tuc2.uc_temp_.get_reply();

  orb_publish(ORB_ID(motor_temp), temp_pub, &motor_temp_raw);

  // ask for new data
  tuc1.uc_temp_.get(*com);
  // tuc2.uc_temp_.get(*com);
  tec1.temp_.get(*com);
  // tec2.temp_.get(*com);
  // int send_ret = com->SendNow();
  // if(send_ret != 0) PX4_WARN("temperature serial send error %d", send_ret);
}

void set_commands(float *actuator_control)
{
  // parameters, for convenience
  double prop_max_speed = parameters[PROP_MAX_SPEED].value;
  double prop_min_pulse = parameters[PROP_MIN_PULSE].value;
  // double prop_max_pulse = parameters[PROP_MAX_PULSE].value;
  double prop_max_voltage = parameters[PROP_MAX_VOLTAGE].value;
  double voltage_coef = parameters[VOLTAGE_COEF].value;
  double pulse_max_coef = parameters[PULSE_MAX_COEF].value;

  // actuator 4 vector
  thrust  = actuator_control[3];
  x_roll    = actuator_control[0];
  y_pitch   = actuator_control[1];
  z_yaw     = actuator_control[2];

  is_coast = false;
  double velocity = 0, delta = 0, volts = 0;
  int sign = 1;

  if ((mode == MODE_FLIGHT)) {
      // Vehicle behavior goes here
      velocity  = thrust * prop_max_speed;
      volts = fmin(velocity / voltage_coef, prop_max_voltage);

      amplitude = pow(x_roll * x_roll + y_pitch * y_pitch, voltage_exponent/2);
      amplitude = fmin(pulse_max_coef, amplitude)*volts;
      amplitude = (amplitude > prop_min_pulse) ? amplitude : 0 ;
      // amplitude = (amplitude < prop_max_pulse) ? amplitude : prop_max_pulse;
      amplitude = (amplitude + volts < prop_max_voltage) ? amplitude : prop_max_voltage - volts;
      // phase = atan2(y_pitch, x_roll);
      phase = atan2(-y_pitch, x_roll);
      z_yaw = 0;
      delta = z_yaw/2;

  } else if (mode == MODE_ROLL) {
      sign = -1;
      phase = 0;
      amplitude = 0;
      velocity = y_pitch * parameters[MAX_ROLL_VEL].value;
      volts = fmin(velocity / voltage_coef, prop_max_voltage);
      delta = x_roll/2;
      if (fabs(z_yaw) < parameters[MIN_ROLL_VEL].value)
          z_yaw = 0;
  }

  if(amplitude < prop_min_pulse) {
      amplitude = 0;
      phase = 0;
  }

  if(iq_control_mode == IQ_CONTROL_VOLTS){
    pmc1.ctrl_volts_.set(*com,-(volts - delta / voltage_coef)); // INDEX_THROTTLE = 3, INDEX_YAW = 2
    pmc2.ctrl_volts_.set(*com,sign*(volts + delta / voltage_coef)); // INDEX_THROTTLE = 3, INDEX_YAW = 2
  } else {
    pmc1.ctrl_velocity_.set(*com,-(velocity - delta)); // INDEX_THROTTLE = 3, INDEX_YAW = 2
    pmc2.ctrl_velocity_.set(*com,sign*(velocity + delta)); // INDEX_THROTTLE = 3, INDEX_YAW = 2
  }

  vsc1.phase_.set(*com, phase - parameters[MOTOR_PHASE_UP].value);
  vsc1.amplitude_.set(*com, amplitude);

  vsc2.phase_.set(*com, phase - parameters[MOTOR_PHASE_DOWN].value);
  vsc2.amplitude_.set(*com, amplitude);

  // iq_motors_state
  mean_velocity[0] = -(velocity - delta);
  mean_velocity[1] = sign*(velocity + delta);
  mean_volts[0] = -(volts - delta / voltage_coef);
  mean_volts[1] = sign*(volts + delta / voltage_coef);
  pulse_volts[0] = amplitude;
  pulse_volts[1] = amplitude;
  pulse_phase[0] = phase - parameters[MOTOR_PHASE_UP].value;
  pulse_phase[1] = phase - parameters[MOTOR_PHASE_DOWN].value;

}

// Send speed commands to the motors, and ask for temperature at the same time
int send_commands(float *actuator_control) {
  //get_temperature();
  if (is_armed) set_commands(actuator_control);

  return com->SendNow();
}

// put motors in coast mode
void enter_coast_mode() {
    pmc1.ctrl_coast_.set(*com);
    pmc2.ctrl_coast_.set(*com);
    int send_ret = com->SendNow();
    if(send_ret != 0) PX4_WARN("serial send error %d, can't put motors on coast mode!", send_ret);
    else PX4_INFO("Motors put in coast mode");
    is_coast = true;
}

// restart iq motors
int iq_reboot(float timeout, int sleep_time){
    //usleep(sleep_time);
    com->Flush();
    com->SendNow();
    usleep(sleep_time);
    pmc1.timeout_.set(*com, timeout);
    pmc2.timeout_.set(*com, timeout);
    sys.reboot_program_.set(*com);
    com->SendNow();
    usleep(sleep_time);
    PX4_INFO("Motors modules (re)booted");
    return 0;
}

void set_motors_state_msg(int send_ret)
{
  iq_motors_state_raw.connection_state = send_ret;
  iq_motors_state_raw.timestamp = hrt_absolute_time();
  iq_motors_state_raw.tec[0] = motor_temp_raw.tec[0];
  iq_motors_state_raw.tec[1] = motor_temp_raw.tec[1];
  iq_motors_state_raw.tuc[0] = motor_temp_raw.tuc[0];
  iq_motors_state_raw.tuc[1] = motor_temp_raw.tuc[1];
  iq_motors_state_raw.iq_control_mode = iq_control_mode;
  iq_motors_state_raw.mean_velocity[0] = mean_velocity[0];
  iq_motors_state_raw.mean_velocity[1] = mean_velocity[1];
  iq_motors_state_raw.mean_volts[0] = mean_volts[0];
  iq_motors_state_raw.mean_volts[1] = mean_volts[1];
  iq_motors_state_raw.pulse_volts[0] = pulse_volts[0];
  iq_motors_state_raw.pulse_volts[1] = pulse_volts[1];
  iq_motors_state_raw.pulse_phase[0] = pulse_phase[0];
  iq_motors_state_raw.pulse_phase[1] = pulse_phase[1];
}

/**
 * send commands
 */
int publish_motors_state(int send_ret)
{
  set_motors_state_msg(send_ret);
  orb_publish(ORB_ID(iq_motors_state), iq_motors_state_pub, &iq_motors_state_raw);

	// lazily publish _actuators only once available
	if (iq_motors_state_pub != nullptr) {
		return orb_publish(ORB_ID(iq_motors_state), iq_motors_state_pub, &iq_motors_state_raw);

	} else {
		iq_motors_state_pub = orb_advertise(ORB_ID(iq_motors_state), &iq_motors_state_raw);

		if (iq_motors_state_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

void set_debug_vect_msg()
{
  debug_vect_raw.x = mean_volts[0];
  debug_vect_raw.y = pulse_volts[0];
  debug_vect_raw.z = pulse_phase[0];
}

/**
 * send commands
 */
int publish_debug_vect()
{
  set_debug_vect_msg();
  orb_publish(ORB_ID(debug_vect), debug_vect_pub, &debug_vect_raw);

	// lazily publish _actuators only once available
	if (iq_motors_state_pub != nullptr) {
		return orb_publish(ORB_ID(debug_vect), debug_vect_pub, &debug_vect_raw);

	} else {
		debug_vect_pub = orb_advertise(ORB_ID(debug_vect), &debug_vect_raw);

		if (debug_vect_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

/**
 ***************************************
 * PARAMETERS FUNCTIONS
 ***************************************
 */

// load single parameter
int parameter_load(parameter *param, bool print_to_console){
    param_t param_handle;
    param_handle = param_find(param->name);
    if (param_handle != PARAM_INVALID){
        float param_f;
        param_get(param_handle, &param_f);
        param->value = (double)param_f;
        if (print_to_console)
          PX4_INFO("%s set to %f", param->name, param->value);
    } else {
        PX4_WARN("%s parameter invalid", param->name);
        return -1;
    }
    return 0;
}

// load single parameter and load it to file
int parameter_load(parameter *param, FILE *fptr, bool print_to_console){
  int ret = parameter_load(param, print_to_console);
  if(ret == 0)
    fprintf(fptr, "%s = %f\n", param->name, param->value);
  return ret;
}

// load all parameters
int parameter_load_all(parameter *params, int n_parameters, bool print_to_console){
  PX4_INFO("Loading PARAMS list, %d parameters",N_PARAMETERS);
  int check = 0;
  for(int i = 0; i < n_parameters; i++){
    check = check + parameter_load(&params[i], print_to_console);
  }
  if(check != 0){
    PX4_WARN("%d PARAMS list could NOT be Loaded",check);
    return 2;
  }
  PX4_INFO("PARAMS list Loaded");
  return 0;
}

// print single parameter on console
int parameter_print(parameter param){
  PX4_INFO("%s set to %f", param.name, param.value);
  return 0;
}

// print single parameter on file
int parameter_print(parameter param, FILE *fptr){
  fprintf(fptr, "%s = %f\n", param.name, param.value);
  return 0;
}

// print all parameters on console
int parameter_print_all(parameter *params, int n_parameters){
  for (int i = 0; i < n_parameters; i++){
    parameter_print(params[i]);
  }
  return 0;
}

// print all parameters on file
int parameter_print_all(parameter *params, int n_parameters, FILE* fptr){
  for (int i = 0; i < n_parameters; i++){
    parameter_print(params[i], fptr);
  }
  return 0;
}


