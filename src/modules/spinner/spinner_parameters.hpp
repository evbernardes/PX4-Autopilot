#ifndef SPINNER_PARAMETERS_HPP
#define SPINNER_PARAMETERS_HPP

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
#define SPIN_KP 3
#define SPIN_KD 4
#define SPIN_KS 5
#define SPIN_KC 6
#define SPIN_KI 7
#define SPIN_ANG_MAX_DEG 8
#define SPIN_JXX 9
#define SPIN_JYY 10
#define SPIN_JZZ 11
#define PROP_JZZ 12
#define PROP_HEIGHT 13
#define PROP_K_THRUST 14
#define PROP_K_RATIO 15
#define MAX_INCLINATION 16
#define ANG_VEL_RATIO 17
#define PROP_MAX_INC 18
#define MAX_ALPHA_DEG 19
#define N_PARAMETERS 20


static parameter params[] = {
  {.name = (char*)"PROP_MAX_SPEED",   .value = 0.0},
  {.name = (char*)"PROP_MAX_VOLTAGE",   .value = 0.0},
  {.name = (char*)"VOLTAGE_COEF",   .value = 0.0},
  {.name = (char*)"SPIN_KP",   .value = 0.0},
  {.name = (char*)"SPIN_KD",   .value = 0.0},
  {.name = (char*)"SPIN_KS",   .value = 0.0},
  {.name = (char*)"SPIN_KC",   .value = 0.0},
  {.name = (char*)"SPIN_KI",   .value = 0.0},
  {.name = (char*)"SPIN_ANG_MAX_DEG",   .value = 0.0},
  {.name = (char*)"SPIN_JXX",   .value = 0.0},
  {.name = (char*)"SPIN_JYY",   .value = 0.0},
  {.name = (char*)"SPIN_JZZ",   .value = 0.0},
  {.name = (char*)"PROP_JZZ",   .value = 0.0},
  {.name = (char*)"PROP_HEIGHT",   .value = 0.0},
  {.name = (char*)"PROP_K_THRUST",   .value = 0.0},
  {.name = (char*)"PROP_K_RATIO",   .value = 0.0},
  {.name = (char*)"MAX_INCLINATION",   .value = 0.0},
  {.name = (char*)"ANG_VEL_RATIO",   .value = 0.0},
  {.name = (char*)"PROP_MAX_INC",   .value = 0.0},
  {.name = (char*)"MAX_ALPHA_DEG",   .value = 0.0}};




// static double v_coef = 0.01f, max_speed_value = 0.01, velocity = 0.01f;
// static double pid_p = 0.01f, pid_d = 0.01f, pid_i = 0.01f;
// static double Jxx = 0.01f, Jyy = 0.01f, Jzz = 0.01f, Jp = 0.01f;
// static double inclination_max_deg = 0.01f;
// static double prop_height = 0.01f, prop_k_thrust = 0.01f, prop_k_torque = 0.01f, prop_kk = 0.01f;


#endif //SPINNER_PARAMETERS_HPP
