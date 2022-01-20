/**
 ***************************************
 * SPEED-PULSE TEST THREAD MAIN
 ***************************************
 */
int iqtest_speedpulse_thread_main(int argc, char *argv[])
{
  char test_type_name[] = "speedpulse";
  test_startup(test_type_name);
  i = 0;
  j = -1;
  speed = 0.0;
  pulse = 0.0;

  while(!thread_should_exit){

    if((stop - start) > step_time * 1000000.0){
      if((j == pulse_n - 1) & (i == speed_n - 1)){
          thread_should_exit = true;
      } else if (j == pulse_n - 1) {
          j = 0;
          i++;
      } else {
          j++;
      }

      if(j > -1){ // motor stopped for first test
          if(speed_n == 1 || float_eq(speed_max, speed_min))
              speed = speed_min;
          else
              speed = (speed_max - speed_min)*i/(speed_n-1) + speed_min;

          if(pulse_n == 1 || float_eq(pulse_max, pulse_min))
              pulse = pulse_min;
          else
              pulse = (pulse_max - pulse_min)*j/(pulse_n-1) + pulse_min;
      }

      // PX4_INFO("Rotor speed = %f (%d/%d) \t Pulse voltage = %f (%d/%d)",speed*prop_max_speed.value, i+1,speed_n,pulse*prop_max_pulse.value, j+1, pulse_n);
      if (print_console) PX4_INFO("Rotor speed = %f (%d/%d) \t Pulse voltage = %f (%d/%d)",
        speed*parameters[PROP_MAX_SPEED].value, i+1,speed_n,pulse*parameters[PROP_MAX_PULSE].value, j+1, pulse_n);
      start = hrt_absolute_time();
    }
    publish_actuators();
    stop = hrt_absolute_time();
    plot_i = speed_n*i + j;
    write_test_line();

  }
  test_finish();
  return 0;
}

/**
 ***************************************
 * SPEED-PULSE TEST THREAD STARTER
 ***************************************
 */
int iqtest_speedpulse_thread_starter(int argc, char *argv[])
{
  if (thread_running) {
      PX4_INFO("already running\n");
      /* this is not an error */
      return 0;
  }
  // default values
  strcpy(test_filename,"speedpulse_test");
  speed_min = 0.3f;
  speed_max = speed_min;
  speed_n = 1;
  pulse_min = 0.0f;
  pulse_max = pulse_min;
  pulse_n = 1;
  phase = 0.0f;
  step_time = 3.0f;
  // plot_step = 5;
  zero_pulse = false;

  switch (argc)
  {
  // case 12: zero_pulse = (bool) atoi(argv[11]);
  case 11: strcpy(test_filename,argv[10]);
  // case 11: plot_step = atoi(argv[10]);
  case 10: step_time = atof(argv[9]);
  case 9: phase = atof(argv[8]) * M_DEG_TO_RAD;
  case 8: pulse_n = atoi(argv[7]);
  case 7: pulse_max = atof(argv[6]);
  case 6: pulse_min = atof(argv[5]);
  case 5: speed_n = atoi(argv[4]);
  case 4: speed_max = atof(argv[3]);
  case 3: speed_min = atof(argv[2]);
  case 2: break;
  // case 1: break;
  default:
      PX4_ERR("usage: iqtest speedpulse [speed_min] [speed_max] [speed_n] [pulse_min] [pulse_max] [pulse_n] [phase] [step_time] [filename]");
      return -1;
  }
  phase_min = phase;
  phase_max = phase;
  thread_should_exit = false;

  daemon_task = px4_task_spawn_cmd("iqtest",
          SCHED_DEFAULT,
          SCHED_PRIORITY_DEFAULT - 2,
          2000,
          iqtest_speedpulse_thread_main,
          (char *const *)NULL);

  PX4_INFO("IQ Test thread started, speed-pulse type");
  return 0;
}

// iqtest speedpulse 0.7 0.7 6 0 0 1 0 10 4fins_attitude_1
