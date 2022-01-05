#ifndef IQ_PARAMETERS_HELPERS_H
#define IQ_PARAMETERS_HELPERS_H

#include <stdio.h>

struct parameter{
    char* name;
    double value;
};

// function prototypes
int parameter_load(parameter *param, bool print_to_console);
int parameter_load(parameter *param, FILE *ftpr, bool print_to_console);
int parameter_load_all(parameter *params, int n_parameters, bool print_to_console);
int parameter_print(parameter param);
int parameter_print(parameter param, FILE *fptr);
int parameter_print_all(parameter *params, int n_parameters);
int parameter_print_all(parameter *params, int n_parameters, FILE* fptr);

#define PROP_MAX_SPEED 0
#define PROP_MAX_VOLTAGE 1
#define VOLTAGE_COEF 2
#define PULSE_MAX_COEF 3
#define PROP_MIN_PULSE 4
#define PROP_MAX_PULSE 5
#define PROP_MAX_YAW 6
#define MIN_ROLL_VEL 7
#define MAX_ROLL_VEL 8
#define MAX_ROLL_INC 9
#define MOTOR_PHASE_UP 10
#define MOTOR_PHASE_DOWN 11
#define TEMP_WARNING 12
#define N_PARAMETERS 13

static parameter parameters[] = {
  {.name = (char*)"PROP_MAX_SPEED",   .value = 0.0},
  {.name = (char*)"PROP_MAX_VOLTAGE",   .value = 0.0},
  {.name = (char*)"VOLTAGE_COEF",   .value = 0.0},
  {.name = (char*)"PULSE_MAX_COEF",   .value = 0.0},
  {.name = (char*)"PROP_MIN_PULSE",   .value = 0.0},
  {.name = (char*)"PROP_MAX_PULSE",   .value = 0.0},
  {.name = (char*)"PROP_MAX_YAW",   .value = 0.0},
  {.name = (char*)"MIN_ROLL_VEL",   .value = 0.0},
  {.name = (char*)"MAX_ROLL_VEL",   .value = 0.0},
  {.name = (char*)"MAX_ROLL_INC",   .value = 0.0},
  {.name = (char*)"MOTOR_PHASE_UP",   .value = 0.0},
  {.name = (char*)"MOTOR_PHASE_DOWN",   .value = 0.0},
  {.name = (char*)"TEMP_WARNING",   .value = 0.0}};

#endif // IQ_PARAMETERS_HELPERS_H
