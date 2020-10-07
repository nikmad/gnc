#ifndef FUNCTIONS_VTOL_H
#define FUNCTIONS_VTOL_H

#include "parameters_vtol.h"

//_______________________________________________________
// declaring functions 

//extern float absolut_(float);
//extern float modulo_(float, float);
//extern float sign_(float);
//
//extern struct force_n_moments forces_moments(struct states, struct actuators, struct wnd);
//extern struct states vtol_dynamics(struct states, struct force_n_moments);
//extern struct states rk4(struct states, struct force_n_moments);
//extern struct state_rates sixDOF(float *, float *);

float absolut_(float);
float modulo_(float, float);
float sign_(float);

struct force_n_moments forces_moments(struct states, struct actuators, struct wnd);
struct states vtol_dynamics(struct states, struct force_n_moments);
struct states rk4(struct states, struct force_n_moments);
struct state_rates sixDOF(float *, float *);
struct ap_gains gains_autopilot(struct states, struct trim_out, struct actuators, struct trans_funcs*);
struct trans_funcs* transfer_functions(struct states, struct trim_out, struct actuators);
float* autopilot(float [], struct ap_gains);
struct actuator_commands pidloop(float,float,float,float,float,float,float,float,float,float,float);
float sat(float, float);
void true_states(struct states, struct force_n_moments, float *);

#endif
