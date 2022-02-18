/**
 ***************************************
 * FREE TEST THREAD MAIN
 ***************************************
 */
int iqtest_free_thread_main(int argc, char *argv[])
{
  char test_type_name[] = "free";
  test_startup(test_type_name);
  PX4_INFO("Starting NOW");

  // send absolutely nothing
  pulse = 0;
  phase = 0;
  speed = 0;

  while(!thread_should_exit){

    if((stop - start) > step_time * 1000000.0){
          thread_should_exit = true;
    }

    publish_actuators();
    stop = hrt_absolute_time();
    write_test_line();
    usleep(sleep_time);
  }
  PX4_INFO("Ending NOW");
  test_finish();
  return 0;
}

/**
 ***************************************
 * FREE TEST THREAD STARTER
 ***************************************
 */
int iqtest_free_thread_starter(int argc, char *argv[]){
  if (thread_running) {
      PX4_INFO("already running\n");
      return 0;
  }

  // default values
  step_time = 10.0;
  sleep_time = 50;
  int priority = SCHED_PRIORITY_DEFAULT + 50;

  strcpy(test_filename, "free_test");

  // fall through
  switch (argc)
  {
  case 6: sleep_time = atoi(argv[5]);
  case 5: priority = atoi(argv[4]);
  case 4: strcpy(test_filename,argv[3]);
  case 3: step_time = atof(argv[2]);
  case 2: break;
  // case 1: break;
  default:
      PX4_ERR("usage: iqtest free [test_time] [filename] [priority] [sleep_time]");
      return -1;
  }

  thread_should_exit = false;

  daemon_task = px4_task_spawn_cmd("iqtest",
          SCHED_DEFAULT,
          priority,
          2000,
          iqtest_free_thread_main,
          (char *const *)NULL);

  PX4_INFO("IQ Test thread started, free type");
  return 0;
}
