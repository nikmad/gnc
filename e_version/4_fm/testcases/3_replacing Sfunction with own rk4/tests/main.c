/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/

#include <stdio.h>
#include <math.h>
#include "vtol_parameters.h"
#include "math_util.h"

//_______________________________________________________
// declaring functions 


float * vtol_dynamics(float *, struct force_n_moments);
float * rk4(float *, struct force_n_moments);
struct dot sixDOF(float *, struct force_n_moments);
float absolut_(float);
float modulo_(float, float);
float sign_(float);


//_______________________________________________________
int main()
{
	//float rk4_stepsize = 0.01;

	//struct states states_in = {0,0};
	//struct states states_out, states_prevMemory;
	struct force_n_moments fm_in = {0,0,0, 0,0,0, 0,0,0, 0,0,0};


	int i;
	float t = 0.0;

	float y[2] = {1,1};
	float *yf;

	//float Va = states_in.u; //DUMMY VALUE ONLY FOR TESTING

	FILE *fptr;
	fptr = fopen("nikstates.txt", "w+");

	fprintf(fptr, "%3.3f   %f   %f\n", t, 0.0, 0.0);

	for(i=1; i<3; i++)
	{
		t = i*SIM.rk4_stepsize;
		//fm_in = forces_moments(states_in, delta, _wind);	
	    yf = vtol_dynamics(y, fm_in);
		//               1     2    3    4    5    6    7    8    9     10  11   12   13   14    15    16   17  18   19   20   21
		//               t     pn   pe   pd   u    v    w   phi  theta psi  p    q    r    Va  alpha beta chi delE delA delR delT
		fprintf(fptr, "%3.3f   %f   %f\n", t, *yf, *(yf+1));
		*y = *yf;
   		//fputs("This is testing for fputs...\n", fptr);
	}

	fclose(fptr);
	
	//printf("Fn1: %f and %f\n", vtol.Cnp, states_out.q);
	
	return 0;
}

//_______________________________________________________

float * vtol_dynamics(float *y, struct force_n_moments fm_in)
{
	float *yf;

	yf = rk4(y, fm_in);

	return yf;
}

//_______________________________________________________

float * rk4(float *y, struct force_n_moments fm_in)
{
	int i;
	int num_states = 2;
	//struct states states_out = {0,0,0,0,0,0,0,0,0,0,0,0};

	//struct states y = states_in;
	//struct states yt;
	//struct force_n_moments uu = fm_in;
	struct dot K1_struct, K2_struct, K3_struct, K4_struct;
	
	float yt[2];

	float h = SIM.rk4_stepsize;

	float uu[] = {	fm_in.fx,
					fm_in.fy,
					fm_in.fz,
					fm_in.l,
					fm_in.m,
					fm_in.n
				};

	//_______________________________________________________
	// K1: following evaluation gives the conventional textbook RK4 term K3
	// - nkm


	K1_struct = sixDOF(y, fm_in);
	float K1[2] = {K1_struct.u, K1_struct.v};
	//struct state_rates K1_struct = {0,0,0,0,0,0,0,0,0,0,0,0};

	for (i=0; i<num_states; ++i)
	{
	    yt[i] = y[i] + (h/2) * K1[i];
	}


	//_______________________________________________________
	// K2: following evaluation gives the conventional textbook RK4 term K2
	// - nkm
	K2_struct = sixDOF(yt,fm_in);
	float K2[2] = {K2_struct.u, K2_struct.v};
	//struct state_rates K2_struct  = {0,0,0,0,0,0,0,0,0,0,0,0};

	for (i=0; i<num_states; ++i)
	{
		yt[i] = y[i] + (h/2) * K2[i];
	}

	//_______________________________________________________
	// K3: following evaluation gives the conventional textbook RK4 term K3
	// - nkm
	K3_struct = sixDOF(yt,fm_in);
	float K3[2] = {K3_struct.u, K3_struct.v};
	//struct state_rates K3_struct = {0,0,0,0,0,0,0,0,0,0,0,0};

	for (i=0; i<num_states; ++i)
	{
		yt[i] = y[i] + h * K3[i];
	}

	//_______________________________________________________
	// K4: following evaluation gives the conventional textbook RK4 term K4
	// - nkm
	K4_struct = sixDOF(yt,fm_in);
	float K4[2] = {K4_struct.u, K4_struct.v};
	//struct state_rates K4_struct = {0,0,0,0,0,0,0,0,0,0,0,0};


	//_______________________________________________________
		
	for (i=0; i<num_states; ++i)
	{
		// The following line gives: u_j+1 = u_j + 1/6*(K1 + 2*(K2+K3) + K4)
		// - nkm
	    y[i] = y[i] + (h/6)*( K1[i] + 2.0*( K2[i] + K3[i] ) + K4[i] );
	}

	return y;
}
//_______________________________________________________

struct dot sixDOF(float *y, struct force_n_moments fm_in)
{

    float u = *y;
    float v = *(y+1);

    float udot, vdot;

    udot = v;
    vdot = -4*u-2*v;

   struct dot body_rates = {udot, vdot};

	return body_rates;
}

//__________________________________________________________



//__________________________________________________________

float absolut_(float x)
{
	float a;
	if(x<0.0) {a = -x;}
	else {a = x;}
	return a;
}

//__________________________________________________________

float modulo_(float x, float y)
{
	float a;
	if(y == 0.0) {a = x;}
	else{a = x - (y*floor(x/y));}
	return a;
}

//__________________________________________________________

float sign_(float x)
{
	int a;
	if(x<0) a = -1;
	else a = 1;
	return a;
}
