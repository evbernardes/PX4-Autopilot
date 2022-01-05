/*
 * iqinetics_params.c
 *
 *  Created on: Aug 14, 2017
 *      Author: Matthew Piccoli
 */

/**
 * Propeller max voltage
 *
 * The propeller's maximum voltage speed
 *
 * @unit V
 * @min 0
 * @max 5
 * @decimal 1
 * @increment 100
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_VOLTAGE, 4.0f);

/**
 * Propeller speed / voltage coefficient
 *
 * The propeller's maximum voltage speed
 *
 * @min 180
 * @max 230
 * @decimal 1
 * @increment 100
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(VOLTAGE_COEF, 202.0f);

/**
 * Propeller min pulse
 *
 * The minimum pulse voltage amplitude (below that, amplitude goes to zero)
 *
 * @unit V
 * @min 0
 * @max 11.1
 * @decimal 1
 * @increment 0.1
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MIN_PULSE, 0.1f);

/**
 * Propeller max pulse
 *
 * The maximum pulse voltage amplitude
 *
 * @unit V
 * @min 0
 * @max 11.1
 * @decimal 1
 * @increment 0.1
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_PULSE, 3.2f);

/**
 * Propeller max pulse/speed percentage
 *
 * The maximum pulse voltage amplitude
 *
 * @min 0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PULSE_MAX_COEF, 0.8f);

/**
 * Propeller max speed
 *
 * The propeller's maximum average speed
 *
 * @unit rad/s
 * @min 0
 * @max 2800
 * @decimal 1
 * @increment 100
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_SPEED, 650.0f);

/**
 * Propeller max yaw
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad/s
 * @min 0
 * @max 2800
 * @decimal 1
 * @increment 10
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_YAW, 500.0f);


/**
 * Rolling mode minimum velocity
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(MIN_ROLL_VEL, 20.0f);

/**
 * Rolling mode maximum velocity
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad/s
 */
PARAM_DEFINE_FLOAT(MAX_ROLL_VEL, 120.0f);

/**
 * Rolling mode roll angle safety
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad
 */
PARAM_DEFINE_FLOAT(MAX_ROLL_INC, 0.78f); // pi / 4

/**
 * Phase of up motor
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad
 */
PARAM_DEFINE_FLOAT(MOTOR_PHASE_UP, -1.570796f); // 0

/**
 * Phase of down motor
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad
 */
PARAM_DEFINE_FLOAT(MOTOR_PHASE_DOWN, 0.0f); // 0

/**
 * Temperature of warning
 *
 *
 *
 */
PARAM_DEFINE_FLOAT(TEMP_WARNING, 35.0f); // 0
