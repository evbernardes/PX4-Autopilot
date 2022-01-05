/**
 ***************************************
 * PHASE TEST THREAD MAIN
 ***************************************
 */
int iqtest_phase_thread_main(int argc, char *argv[])
{
  double phase_test_pulse = pulse;
  double S=0,C=0;
  double qx=0, qy=0;
  pulse = 0;
  phase = 0;
  char test_type_name[] = "phase";
  test_startup(test_type_name);
  if (print_console) PX4_INFO("Test speed = %.2frad/s, Test pulse = %.2fV",(double) speed * parameters[PROP_MAX_SPEED].value, (double) pulse*parameters[PROP_MAX_PULSE].value);

  int i_ = -1;
  if (centering)
    phase_n = 2*phase_n;

  while(!thread_should_exit){

      if((stop - start) > (double) step_time * 1000000.0){
          if(i_ == phase_n - 1){
              thread_should_exit = true;
          } else {
              i_++;

              if(i_ > 0){
                  double phase_acc = atan2(-qy,qx);
                //   float diff = (phase_acc, phase);
                  double diff = phase_acc - phase;
                  S += sin(diff)/phase_n;
                  C += cos(diff)/phase_n;
              }

              phase = phase_min + i_ * (phase_max - phase_min)/phase_n;

              if (centering){
                if (i_ % 2 == 0){
                  pulse = phase_test_pulse;
                  if (print_console) PX4_INFO("Phase = %.2f (%d/%d), time = %.2f", (double) phase * (double) M_RAD_TO_DEG_F, i_/2+1,phase_n/2, (double) start/1000000.0);
                } else {
                  pulse = 0;
                  if (print_console) PX4_INFO("Centering..., time = %.2f", start/1000000.0);
                }
              } else {
                pulse = phase_test_pulse;
                if (print_console) PX4_INFO("Phase = %.2f (%d/%d), time = %.2f",(double) phase * (double) M_RAD_TO_DEG_F, i_+1,phase_n, (double) start/1000000.0);
              }

              start = hrt_absolute_time();
          }
      }

      publish_actuators();
      stop = hrt_absolute_time();
      plot_i = i_;
      write_test_line();
  }

  test_finish();
  double norm = sqrt(S*S + C*C);
  S = S/norm;
  C = C/norm;
  phase = (double) atan2(S,C) * M_RAD_TO_DEG;
  PX4_INFO("Final calculated phase = %.2f",phase);
  return 0;
}

/**
 ***************************************
 * PHASE TEST THREAD STARTER
 ***************************************
 */
int iqtest_phase_thread_starter(int argc, char *argv[]){
  if (thread_running) {
      PX4_INFO("already running\n");
      /* this is not an error */
      return 0;
  }
  strcpy(test_filename, "test");
  phase_min = 0.;
  phase_max = 2.0f * M_PI_F;
  phase_n = 4;
  step_time = 2.0;
  centering = false;
  speed = 0.5;
  pulse = 0.6;

  switch (argc)
  {
  case 10: strcpy(test_filename, argv[9]);
  case 9: pulse = atof(argv[8]);
  case 8: speed = atof(argv[7]);
  case 7: centering = (bool) atoi(argv[6]);
  case 6: step_time = atof(argv[5]);
  case 5: phase_n = atoi(argv[4]);
  case 4: phase_max = atof(argv[3]) * M_DEG_TO_RAD;
  case 3: phase_min = atof(argv[2]) * M_DEG_TO_RAD;
  case 2: break;
  // case 1: break;
  default:
      PX4_ERR("usage: iqtest phase [phase_min] [phase_max] [phase_n] [step_time] [centering] [speed] [pulse] [filename]");
      return -1;
  }

  if (phase_n < 1){
      phase_n = 1;
  }
  speed_min = speed;
  speed_max = speed;
  pulse_min = pulse;
  pulse_max = pulse;
  thread_should_exit = false;

  daemon_task = px4_task_spawn_cmd("iqtest",
          SCHED_DEFAULT,
          SCHED_PRIORITY_DEFAULT - 2,
          2000,
          iqtest_phase_thread_main,
          (char *const *)NULL);

  return 0;
}
