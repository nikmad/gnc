#ifndef FUNCTIONS_VTOL_H
#define FUNCTIONS_VTOL_H

#include<stdlib.h>

#ifndef MYLIB_CONSTANTS_H
	#include "vtol_parameters.h"
#define MYLIB_CONSTANTS_H
#endif

//_______________________________________________________
// declaring functions 

extern float absolut_(float);
extern float modulo_(float, float);
extern float sign_(float);

extern struct force_n_moments forces_moments(struct states, struct actuators, struct wnd);
extern struct states vtol_dynamics(struct states, struct force_n_moments);
extern struct states rk4(struct states, struct force_n_moments);
extern struct state_rates sixDOF(float *, float *);

#endif
