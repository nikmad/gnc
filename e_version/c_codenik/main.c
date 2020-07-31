/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/

#include <stdio.h>
#include "vtol_parameters.h"

//_______________________________________________________
// declaring functions 

float* vtol_dynamics(struct states, struct, struct, float);
float* rk4(struct, struct, struct, float);

//_______________________________________________________
int main()
{
	float rk4_stepsize = 0.01;

	vtol_dynamics(states_in, fm_in, vtol, rk4_stepsize);
}

//_______________________________________________________

vtol_dynamics(states_in, fm_in, vtol, rk4_stepsize)
{
	states_out = rk4(states_in, fm_in, vtol, rk4_stepsize);
	
	return states_out;
}

//_______________________________________________________

rk4(states_in, fm_in, vtol, rk4_stepsize)
{
	int num_states = 12;

	yt = 

	return states_out;
}

//_______________________________________________________