/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/
//#define EXTERN
//#define VTOL

#include <stdio.h>
#include <math.h>
#include "parameters_vtol.h"
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
	
	struct trans_funcs* tf;
	tf = (struct trans_funcs*)malloc(8*sizeof(struct trans_funcs));
	
	int i;
	float t = 0.0, t_tot = 50.0;

	float chi = 0.0;

	struct states x_trim = {0, 0, 0, 34.99, 0, 0.082, -0.00022, -0.26, 0, 0, 0, 0};
	struct trim_out y_trim = {35, 0.0023, 0};
	struct actuators u_trim = {0.0072, 0.0012, -0.00019, 0.85};

	// Initial states set to trim values
	states_in = x_trim;
	states_in.pn = 0;
	states_in.pe = 0;
	states_in.pd = -140;

	tf = transfer_functions(x_trim, y_trim, u_trim);

	FILE *fptr_tf;
	fptr_tf = fopen("transfer_functions.txt", "w+");
	
	fprintf(fptr_tf, "    		b0\t\t b1\t\t b2\t\t b3\t\t a0\t\t a1\t\t a2\t\t a3\n\n");
	fprintf(fptr_tf, "tf_phiDelA  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",    tf->b0,     tf->b1,     tf->b2,     tf->b3,     tf->a0,     tf->a1,     tf->a2,     tf->a3);
	fprintf(fptr_tf, "tf_chiPhi  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+1)->b0, (tf+1)->b1, (tf+1)->b2, (tf+1)->b3, (tf+1)->a0, (tf+1)->a1, (tf+1)->a2, (tf+1)->a3);
	fprintf(fptr_tf, "tf_thetaDelE  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+2)->b0, (tf+2)->b1, (tf+2)->b2, (tf+2)->b3, (tf+2)->a0, (tf+2)->a1, (tf+2)->a2, (tf+2)->a3);
	fprintf(fptr_tf, "tf_hTheta  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+3)->b0, (tf+3)->b1, (tf+3)->b2, (tf+3)->b3, (tf+3)->a0, (tf+3)->a1, (tf+3)->a2, (tf+3)->a3);
	fprintf(fptr_tf, "tf_hVa  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+4)->b0, (tf+4)->b1, (tf+4)->b2, (tf+4)->b3, (tf+4)->a0, (tf+4)->a1, (tf+4)->a2, (tf+4)->a3);
	fprintf(fptr_tf, "tf_VaDelT  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+5)->b0, (tf+5)->b1, (tf+5)->b2, (tf+5)->b3, (tf+5)->a0, (tf+5)->a1, (tf+5)->a2, (tf+5)->a3);
	fprintf(fptr_tf, "tf_VaTheta 	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+6)->b0, (tf+6)->b1, (tf+6)->b2, (tf+6)->b3, (tf+6)->a0, (tf+6)->a1, (tf+6)->a2, (tf+6)->a3);
	fprintf(fptr_tf, "tf_vDelR  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+7)->b0, (tf+7)->b1, (tf+7)->b2, (tf+7)->b3, (tf+7)->a0, (tf+7)->a1, (tf+7)->a2, (tf+7)->a3);

	
	fclose(fptr_tf);

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

	free(tf);
	
	return 0;
}

