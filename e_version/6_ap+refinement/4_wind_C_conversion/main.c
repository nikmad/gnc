/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/

#include <stdio.h>
#include <math.h>
//#include "vtol_parameters.h"
#include "functions_vtol.h"
#include "math_util.h"


//_______________________________________________________
int main()
{
	//float rk4_stepsize = 0.01;

	struct states states_in = {0,0,0,0,0,0,0,0,0,0,0,0};
	struct states states_out, states_prevMemory;
	struct force_n_moments fm_in = {0,0,0, 0,0,0, 0,0,0, 0,0,0};
	struct actuators delta = {0*PI/180, 0*PI/180, 15*PI/180, 0};
	struct wnd _wind = {0.0000000001,0.0000000001,0.0000000001,0.0000000001,0.0000000001,0.0000000001};
	int i;
	float t = 0.0, t_tot = 50.0;

	//float Va = states_in.u; //DUMMY VALUE ONLY FOR TESTING

	float chi = 0.0;


	FILE *fptr;
	fptr = fopen("nikstates.txt", "w+");

	fprintf(fptr, "%3.3f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", t, states_in.pn, states_in.pe, states_in.pd, states_in.u, states_in.v, states_in.w, states_in.phi, states_in.theta, states_in.psi, states_in.p, states_in.q, states_in.r, fm_in.Va, fm_in.alpha,fm_in.beta,chi,delta.delta_e,delta.delta_a,delta.delta_r,delta.delta_t);

	for(i=1; i<(int)(t_tot/SIM.rk4_stepsize)+1; i++)
	{
		t = i*SIM.rk4_stepsize;
		fm_in = forces_moments(states_in, delta, _wind);
		states_out = vtol_dynamics(states_in, fm_in);
		chi = (float)atan2((fm_in.Va*sinf(states_out.psi)+fm_in.w_e),(fm_in.Va*cosf(states_out.psi)+fm_in.w_n));
		//               1     2    3    4    5    6    7    8    9     10  11   12   13   14    15   16   17  18   19   20   21
		//               t     pn   pe   pd   u    v    w   phi  theta psi  p    q    r    Va  alpha beta chi delE delA delR delT
		fprintf(fptr, "%3.3f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", t, states_out.pn, states_out.pe, states_out.pd, states_out.u, states_out.v, states_out.w, states_out.phi, states_out.theta, states_out.psi, states_out.p, states_out.q, states_out.r, fm_in.Va, fm_in.alpha, fm_in.beta,chi,delta.delta_e,delta.delta_a,delta.delta_r,delta.delta_t);
		states_in = states_out;
   	}

	fclose(fptr);
	
	return 0;
}

