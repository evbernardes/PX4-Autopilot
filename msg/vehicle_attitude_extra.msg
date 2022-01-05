uint64 timestamp		# time since system start (microseconds)
float32 freq
#float32[4] ned_q			# Quaternion rotation from XYZ body frame to NED earth frame.
#float32[3] ned_xyz			# Euler angles rotation from XYZ body frame to NED earth frame.
#float32[3] ned_zyz			# Euler angles rotation from ZYZ body frame to NED earth frame.
#float32[3] ned_vz
#float32[4] enu_q			# Quaternion rotation from XYZ body frame to ENU earth frame.
#float32[3] enu_xyz			# Euler angles rotation from XYZ body frame to ENU earth frame.
#float32[3] enu_zyz			# Euler angles rotation from ZYZ body frame to ENU earth frame.
#float32[3] enu_vz

uint32 is_ned
uint32 is_thrust_spin
uint32 is_debug
float32[4] q			        # Quaternion rotation from XYZ body frame to NED earth frame.
float32[4] qa			        # Quaternion rotation from XYZ body frame to NED earth frame.
float32[4] qb			        # Quaternion rotation from XYZ body frame to NED earth frame.
float32 theta

float32 pot_energy
float32[4] qdelta			        # Quaternion rotation from XYZ body frame to NED earth frame.

float32[2] swash_deg
float32[3] v
float32[3] e
float32[3] p
float32[3] vd
float32[3] wd
float32[3] w
float32[4] control
float32[3] torque
float32[3] thrust
float32[3] n
float32[3] ne
float32[3] euler_xyz			# Euler angles rotation from XYZ body frame to NED earth frame.
float32[3] euler_zyz			# Euler angles rotation from ZYZ body frame to NED earth frame.

# TOPICS vehicle_attitude_extra spinneratt
