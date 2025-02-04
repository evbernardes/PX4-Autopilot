
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v4
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	TESTING
	UAVCAN_INTERFACES 1

	SERIAL_PORTS
		WIFI:/dev/ttyS0
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2

	DRIVERS
		adc
		barometer # all available barometer drivers
		batt_smbus
		camera_capture
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		dshot
		gps
		heater
		#imu # all available imu drivers
		imu/mpu6000
		imu/mpu9250
		irlock
		lights/blinkm
		lights/rgbled
		lights/rgbled_ncp5623c
		magnetometer # all available magnetometer drivers
		mkblctrl
		optical_flow # all available optical flow drivers
		pca9685
		pwm_input
		pwm_out_sim
		px4fmu
		rc_input
		safety_button
		tap_esc
		telemetry # all available telemetry drivers
		test_ppm
		tone_alarm
		uavcan

	MODULES
		airspeed_selector
		attitude_estimator_q
		battery_status
		camera_feedback
		commander
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		rover_pos_control
		sensors
		sih
		vmount
		vtol_att_control
    spinner_control
        iq

	SYSTEMCMDS
		bl_update
		config
		dumpfile
		esc_calib
		hardfault_log
		i2cdetect
		led_control
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		shutdown
		tests # tests and test runner
		top
		topic_listener
		tune_control
		usb_connected
		ver
		work_queue

	EXAMPLES
		bottle_drop # OBC challenge
		fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		hello
		hwtest # Hardware test
		#matlab_csv_serial
		px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		rover_steering_control # Rover example app
		uuv_example_app

	)
