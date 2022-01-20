/**
 ***************************************
 * SPEED TEST THREAD MAIN
 ***************************************
 */
int iqtest_speed_thread_main(int argc, char *argv[])
{
  char test_type_name[] = "speed";
  test_startup(test_type_name);
  i = 0;
  plot_i = 0;
  speed = 0.0;
  double prop_max_speed = parameters[PROP_MAX_SPEED].value;
  double prop_max_pulse = parameters[PROP_MAX_PULSE].value;
  bool going_backwards = false;
  int total_n = should_go_backwards? speed_n : 2*speed_n - 1;

  while(!thread_should_exit){

    if((stop - start) > step_time * 1000000.0){
      if(i == 0 && going_backwards){
        thread_should_exit = true;

      } else if(i == speed_n - 1){
        if(should_go_backwards and !going_backwards){
          going_backwards = true;
          i--;
        } else {
          thread_should_exit = true;
        }

      } else {
        i = going_backwards ? i - 1 : i + 1;
        plot_i++;
      }

      if(speed_n == 1 || float_eq(speed_max, speed_min))
          speed = speed_min;
      else
          speed = speed_min + i*(speed_max - speed_min)/(speed_n-1);

      if(going_backwards){
        if (print_console) PX4_INFO("Rotor speed = %f (%d/%d) \t Pulse voltage = %f \t going backwards %d",
              speed*prop_max_speed, plot_i+1, total_n, pulse*prop_max_pulse, i);
      } else {
        if (print_console) PX4_INFO("Rotor speed = %f (%d/%d) \t Pulse voltage = %f \t going forwards %d",
              speed*prop_max_speed, plot_i+1, total_n, pulse*prop_max_pulse, i);
      }
      start = hrt_absolute_time();
    }
    publish_actuators();
    stop = hrt_absolute_time();
    write_test_line();
  }
  test_finish();
  return 0;
}

/**
 ***************************************
 * SPEED TEST THREAD STARTER
 ***************************************
 */
int iqtest_speed_thread_starter(int argc, char *argv[]){
  if (thread_running) {
    PX4_INFO("already running\n");
    /* this is not an error */
    return 0;
  }
  // default values
  speed_min = 0.0f;
  speed_max = 0.4f;
  speed_n = 5;
  pulse = 0.0f;
  phase = 0.0f;
  step_time = 1.5f;
  should_go_backwards = true;
  strcpy(test_filename,"speed_test");
  // plot_step = 5;

  switch (argc)
  {
  case 10: strcpy(test_filename,argv[9]);
  case 9: should_go_backwards = (bool) atoi(argv[8]);
  case 8: step_time = atof(argv[7]);
  case 7: phase = atof(argv[6]) * M_DEG_TO_RAD;
  case 6: pulse = atof(argv[5]);
  case 5: speed_n = atoi(argv[4]);
  case 4: speed_max = atof(argv[3]);
  case 3: speed_min = atof(argv[2]);
  case 2: break;
  // case 1: break;
  default:
      PX4_ERR("usage: iqtest speed [speed_min] [speed_max] [speed_n] [pulse] [phase] [step_time] [go_backwards] [filename]");
      return -1;
  }
  phase_min = phase;
  phase_max = phase;
  phase_n = 1;
  pulse_min = pulse;
  pulse_max = pulse;
  pulse_n = 1;
  thread_should_exit = false;

  daemon_task = px4_task_spawn_cmd("iqtest",
          SCHED_DEFAULT,
          SCHED_PRIORITY_DEFAULT - 2,
          2000,
          iqtest_speed_thread_main,
          (char *const *)NULL);

  PX4_INFO("IQ Test thread started, speed type");
  return 0;
}

// usage: iqtest speed [speed_min] [speed_max] [speed_n] [pulse] [phase] [step_time] [go_backwards] [filename]
// iqtest speed 0 0.4 5 0 0 3 1 test_backwards
// iqtest speed 0.5 0.8 10 0 0 6 0 6fins_attitude
// iqtest speed 0.1 0.1 1 0 0 1 0 test
// usage: iqtest phase [phase_min] [phase_max] [phase_n] [step_time] [centering] [speed] [pulse] [filename]
// usage: iqtest speedpulse [speed_min] [speed_max] [speed_n] [pulse_min] [pulse_max] [pulse_n] [phase] [step_time] [filename]



