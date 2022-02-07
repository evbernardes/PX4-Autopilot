# Motor control message
uint64 timestamp			# time since system start (microseconds)
uint64 iq_control_mode
float32[2] mean_velocity
float32[2] mean_volts
float32[2] pulse_volts
float32[2] pulse_phase
float32[2] tec
float32[2] tuc

# TOPICS iq_motors_state
