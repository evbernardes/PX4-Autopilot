void get_all_rands();

/**
 ***************************************
 * RANDOM TEST THREAD MAIN
 ***************************************
 */
int iqtest_random_thread_main(int argc, char *argv[])
{
  char test_type_name[] = "random";
  test_startup(test_type_name);
  srand(start);
  i = 0;

  // speed = get_rand(speed_min, speed_max);
  // pulse = get_rand(pulse_min, pulse_max);
  // phase = get_rand(phase_min, phase_max);
  // step_time = get_rand(step_min, step_max);

  // if (pulse < 0){
  //   pulse = -pulse;
  //   phase = phase + M_PI_F;
  // }
  get_all_rands();

  while(!thread_should_exit){

    if((stop - start) > step_time * 1000000.0){
      if(i == random_n - 1)
          thread_should_exit = true;

      get_all_rands();

      if (phase_alternate && i%2 == 1){
        phase = phase + M_PI;
      }


      // PX4_INFO("Speed = %.2frad/s, Pulse = %.2fV, Phase = %.2fdeg, Time = %.2fs",speed*prop_max_speed.value,pulse*prop_max_pulse.value, phase*180/M_PI_F, step_time);
      if (print_console) PX4_INFO("Speed = %.2frad/s, Pulse = %.2fV, Phase = %.2fdeg, Time = %.2fs",
        speed*parameters[PROP_MAX_SPEED].value,pulse*parameters[PROP_MAX_PULSE].value, phase*M_RAD_TO_DEG, step_time);
      start = hrt_absolute_time();
      i++;
    }
    publish_actuators();
    stop = hrt_absolute_time();
    plot_i = i;
    write_test_line();
  }
  test_finish();
  return 0;
}

/**
 ***************************************
 * RANDOM TEST THREAD STARTER
 ***************************************
 */
int iqtest_random_thread_starter(int argc, char *argv[]){
  if (thread_running) {
      PX4_INFO("already running\n");
      return 0;
  }

  // default values
  speed_min = 0.3;
  speed_max = 0.5;
  pulse_min = 0;
  pulse_max = 0.3;
  step_min = 0.5;
  step_max = 1;
  random_n = 5;
  phase_alternate = false;
  time_pulse_scale = false;
  strcpy(test_filename, "random_test");

  // fall through
  switch (argc)
  {
  case 14: strcpy(test_filename,argv[13]);
  case 13: time_pulse_scale = (bool) atoi(argv[12]);
  case 12: phase_alternate = (bool) atoi(argv[11]);
  case 11: step_max = atof(argv[10]);
  case 10: step_min = atof(argv[9]);
  case 9: phase_max = atof(argv[8]) * M_DEG_TO_RAD;
  case 8: phase_min = atof(argv[7]) * M_DEG_TO_RAD;
  case 7: pulse_max = atof(argv[6]);
  case 6: pulse_min = atof(argv[5]);
  case 5: speed_max = atof(argv[4]);
  case 4: speed_min = atof(argv[3]);
  case 3: random_n = atoi(argv[2]);
  case 2: break;
  // case 1: break;
  default:
      PX4_ERR("usage: iqtest random [N] [speed_min] [speed_max] [pulse_min] [pulse_max] [phase_min] [phase_max] [step_min] [step_max] [phase_alternate] [scale_time] [filename]");
      // PX4_ERR("usage: iqtest random [N] [speed_min] [speed_max] [pulse_min] [pulse_max] [phase_min] [phase_max] [step_min] [step_max] [phase_alternate] [filename]");
      return -1;
  }
  phase_n = random_n;
  speed_n = random_n;
  thread_should_exit = false;

  daemon_task = px4_task_spawn_cmd("iqtest",
          SCHED_DEFAULT,
          SCHED_PRIORITY_DEFAULT - 2,
          2000,
          iqtest_random_thread_main,
          (char *const *)NULL);

  PX4_INFO("IQ Test thread started, random type");
  return 0;
}

//
// Helpers
//

void get_all_rands(){

  speed = get_rand(speed_min, speed_max);
  pulse = get_rand(pulse_min, pulse_max);
  phase = get_rand(phase_min, phase_max);

  if (pulse < 0){
    pulse = -pulse;
    phase = phase + M_PI;
  }

  if(time_pulse_scale){
    step_time = step_min + (pulse_max - pulse)*(step_max - step_min)/(pulse_max - pulse_min);
  } else {
    step_time = get_rand(step_min, step_max);
  }
}

// usage: iqtest random [N] [speed_min] [speed_max] [pulse_min] [pulse_max]
//                           [phase_min] [phase_max] [step_min] [step_max]
//                           [phase_alternate] [time_pulse_scale] [filename]

// iqtest random 30 0.5 0.5 0.3 0.6 0 0 1.0 1.5 1 2021_10_15/R30_v05x05_p03x06_t05x15
// iqtest random 30 0.5 0.5 0.3 0.6 0 0 3.0 1.5 1 2021_10_15/R30_v05x05_p03x06_t05x15

// iqtest random 30 0.5 0.5 0.3 0.8 0 0 3.0 1.5 1 2021_10_15/R30_v05_p03x06_t05x15

// iqtest random 6 0.5 0.6 0.3 0.8 0 0 2.0 2.0 1 0 TETEST
// iqtest random 30 0.6 0.6 0.3 0.8 0 0 2.0 2.0 1 0 R30_v06_p03x08_t20
// iqtest random 30 0.7 0.7 0.3 0.8 0 0 2.0 2.0 1 0 R30_v07_p03x08_t20
// iqtest random 30 0.8 0.8 0.3 0.8 0 0 1.0 2.0 1 1 R30sc_v08_p03x08_t20
// iqtest random 50 0.8 0.8 0.3 0.8 0 0 1.0 1.0 1 0 R50_v08_p03x08_t10

// mv fs/microsd/test/R30_v06_p03x06_t20.dat fs/microsd/test/R30_v06_p03x08_t20.dat
