/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/

#include <stdio.h>
#include <math.h>
#include "vtol_parameters.h"

//_______________________________________________________
// declaring functions 

struct states vtol_dynamics(struct states, struct force_n_moments);
struct states rk4(struct states, struct force_n_moments);
struct state_rates sixDOF(float *, float *);
float absolut_(float);
float modulo_(float, float);
float sign_(float);

//_______________________________________________________
int main()
{
	//float rk4_stepsize = 0.01;

	struct states states_in = {0,0,0,0,0,0,0,0,0,0,0,0};
	struct states states_out, states_prevMemory;
	struct force_n_moments fm_in = {0,0,2.0,0,0,0};
	int i;
	float t;

	//states_in.pn = 0.0;
	//states_in.pe = 0.0;
	//states_in.pd = 0.0;
	//states_in.u = vtol.Va0;
	//states_in.v = 0.0;
	//states_in.w = 0.0;
	//states_in.phi = 0.0;
	//states_in.theta = 0.0;
	//states_in.psi = 0.0;
	//states_in.p = 0.0;
	//states_in.q = 0.0;
	//states_in.r = 0.0;

	//fm_in.fx = 0.0;
	//fm_in.fy = 0.0;
	//fm_in.fz = 0.0;
	//fm_in.l = 0.0;
	//fm_in.m = 0.0;
	//fm_in.n = 0.0;

	FILE *fptr;
	fptr = fopen("nikstates.txt", "w+");

	for(i=0; i<100; i++)
	{
		t = i*0.01;
	    states_out = vtol_dynamics(states_in, fm_in);
		fprintf(fptr, "%3.3f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", t, states_out.pn, states_out.pe, states_out.pd, states_out.u, states_out.v, states_out.w, states_out.phi, states_out.theta, states_out.psi, states_out.p, states_out.q, states_out.r);
		states_in = states_out;
   		//fputs("This is testing for fputs...\n", fptr);
	}

	fclose(fptr);
	
	printf("Fn1: %f and %f\n", vtol.Cnp, states_out.q);
	
	return 0;
}

//_______________________________________________________

struct states vtol_dynamics(struct states states_in, struct force_n_moments fm_in)
{
	struct states states_out;

	states_out = rk4(states_in, fm_in);

	return states_out;
}

//_______________________________________________________

struct states rk4(struct states states_in, struct force_n_moments fm_in)
{
	int i;
	int num_states = 12;
	//struct states states_out = {0,0,0,0,0,0,0,0,0,0,0,0};

	//struct states y = states_in;
	//struct states yt;
	//struct force_n_moments uu = fm_in;
	struct state_rates K1_struct, K2_struct, K3_struct, K4_struct;
	
	float yt[12];

	float h = SIM.rk4_stepsize;

	float y[] =	{	states_in.pn,
					states_in.pe,
					states_in.pd,
					states_in.u,
					states_in.v,
					states_in.w,
					states_in.phi,
					states_in.theta,
					states_in.psi,
					states_in.p,
					states_in.q,
					states_in.r 
				};

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
	K1_struct = sixDOF(y, uu);
	//struct state_rates K1_struct = {0,0,0,0,0,0,0,0,0,0,0,0};

	float K1[] = {	K1_struct.pndot,
					K1_struct.pedot,
					K1_struct.pddot,
					K1_struct.udot,
					K1_struct.vdot,
					K1_struct.wdot,
					K1_struct.phidot,
					K1_struct.thetadot,
					K1_struct.psidot,
					K1_struct.pdot,
					K1_struct.qdot,
					K1_struct.rdot
				};

	for (i=0; i<num_states; ++i)
	{
	    yt[i] = y[i] + h/2 * K1[i];
	}


	//_______________________________________________________
	// K2: following evaluation gives the conventional textbook RK4 term K2
	// - nkm
	K2_struct = sixDOF(yt,uu);
	//struct state_rates K2_struct  = {0,0,0,0,0,0,0,0,0,0,0,0};

	float K2[] = {	K2_struct.pndot,
					K2_struct.pedot,
					K2_struct.pddot,
					K2_struct.udot,
					K2_struct.vdot,
					K2_struct.wdot,
					K2_struct.phidot,
					K2_struct.thetadot,
					K2_struct.psidot,
					K2_struct.pdot,
					K2_struct.qdot,
					K2_struct.rdot
				};

	for (i=0; i<num_states; ++i)
	{
		yt[i] = y[i] + h/2 * K2[i];
	}

	//_______________________________________________________
	// K3: following evaluation gives the conventional textbook RK4 term K3
	// - nkm
	K3_struct = sixDOF(yt,uu);
	//struct state_rates K3_struct = {0,0,0,0,0,0,0,0,0,0,0,0};

	float K3[] = {	K3_struct.pndot,
					K3_struct.pedot,
					K3_struct.pddot,
					K3_struct.udot,
					K3_struct.vdot,
					K3_struct.wdot,
					K3_struct.phidot,
					K3_struct.thetadot,
					K3_struct.psidot,
					K3_struct.pdot,
					K3_struct.qdot,
					K3_struct.rdot
				};

	for (i=0; i<num_states; ++i)
	{
		yt[i] = y[i] + h * K3[i];
	}

	//_______________________________________________________
	// K4: following evaluation gives the conventional textbook RK4 term K4
	// - nkm
	K4_struct = sixDOF(yt,uu);
	//struct state_rates K4_struct = {0,0,0,0,0,0,0,0,0,0,0,0};

	float K4[] = {	K4_struct.pndot,
					K4_struct.pedot,
					K4_struct.pddot,
					K4_struct.udot,
					K4_struct.vdot,
					K4_struct.wdot,
					K4_struct.phidot,
					K4_struct.thetadot,
					K4_struct.psidot,
					K4_struct.pdot,
					K4_struct.qdot,
					K4_struct.rdot
				};


	//_______________________________________________________
		
	for (i=0; i<num_states; ++i)
	{
		// The following line gives: u_j+1 = u_j + 1/6*(K1 + 2*(K2+K3) + K4)
		// - nkm
	    y[i] = y[i] + h/6*( K1[i] + 2.0*( K2[i] + K3[i] ) + K4[i] );
	}

	struct states states_out = 
				{	y[0],
					y[1],
					y[2],
					y[3],
					y[4],
					y[5],
					y[6],
					y[7],
					y[8],
					y[9],
					y[10],
					y[11]			
				};

	return states_out;
}
//_______________________________________________________

struct state_rates sixDOF(float *states_in, float* fm_in)
{
	float pndot;
	float pedot;
	float pddot;
	float udot;
	float vdot;
	float wdot;
	float phidot;
	float thetadot;
	float psidot;
	float pdot;
	float qdot;
	float rdot;

	float pn 	= *states_in;
	float pe 	= *(states_in+1);
	float pd 	= *(states_in+2);
	float u 	= *(states_in+3);
	float v 	= *(states_in+4);
	float w 	= *(states_in+5);
	float phi 	= *(states_in+6);
	float theta = *(states_in+7);
	float psi 	= *(states_in+8);
	float p 	= *(states_in+9);
	float q 	= *(states_in+10);
	float r 	= *(states_in+11);

    float fx    = *fm_in;
    float fy    = *(fm_in+1);
    float fz    = *(fm_in+2);
    float ell   = *(fm_in+3);
    float m     = *(fm_in+4);
    float n     = *(fm_in+5);

    float mass = vtol.m;
    float Ix = vtol.Jx;
    float Iy = vtol.Jy;
    float Iz = vtol.Jz;
    float Ixz= vtol.Jxz;

    float c0=vtol.gamma0; 
    float c1=vtol.gamma1;    
    float c2=vtol.gamma2;                                              
    float c3=vtol.gamma3;
    float c4=vtol.gamma4;
    float c5=vtol.gamma5;
    float c6=vtol.gamma6;
    float c7=vtol.gamma7;
    float c8=vtol.gamma8;

    printf("value of theta = %f\n", theta);

    pndot = u*cosf(theta)*cosf(psi) + v*(sinf(phi)*sinf(theta)*cosf(psi)-cosf(phi)*sinf(psi)) + w*(cosf(phi)*sinf(theta)*cosf(psi)+sinf(phi)*sinf(psi));
    pedot = u*cosf(theta)*sinf(psi) + v*(sinf(phi)*sinf(theta)*sinf(psi)+cosf(phi)*cosf(psi)) + w*(cosf(phi)*sinf(theta)*sinf(psi)-sinf(phi)*cosf(psi));
    pddot = -u*sinf(theta) + v*sinf(phi)*cosf(theta) + w*cosf(phi)*cosf(theta);

    udot = r*v-q*w + fx/mass;
    vdot = p*w-r*u + fy/mass;
    wdot = q*u-p*v + fz/mass;

	float vSmall = 0.005;

	//____________________________________________________________
	// nkm - "Around 2.8648 deg (in rad 0.05) below and above 90 deg
	// has to be discarded for sec() and tan() functions because
	// in this zone, increase in sec(theta) and tan(theta) for a
	// small rise in theta is huge. So we discard the zone (90-2.86)
	// to (90+2.86) i.e., 87.14 to 92.86 deg".

	float sensitive90zone = 0.05; 
	
	//____________________________________________________________

	float theta1 = (float)modulo_(absolut_(theta),2*pi);
	float theta2 = (float)floor(absolut_(theta)/(2*pi))*(2*pi);

	if ((floor(theta1/(pi/2))==0) && (pi/2-theta1 < sensitive90zone))
		{theta = sign_(theta)*(pi/2 - sensitive90zone + theta2);}
	else if ((floor(theta1/(pi/2))==1) &&   (theta1-pi/2 < sensitive90zone))
		{theta = sign_(theta)*(pi/2 + sensitive90zone + theta2);}
	else if ((floor(theta1/(pi/2))==2) && (3*pi/2-theta1 < sensitive90zone))
		{theta = sign_(theta)*(3*pi/2 - sensitive90zone + theta2);}
	else if ((floor(theta1/(pi/2))==3) && (theta1-3*pi/2 < sensitive90zone))
		{theta = sign_(theta)*(3*pi/2 + sensitive90zone + theta2);}

	phidot = p + q*sinf(phi)*tanf(theta) + r*cosf(phi)*tanf(theta);
	thetadot = q*cosf(phi)-r*sinf(phi);
	//psidot = q*sinf(phi)*sec(theta) + r*cosf(phi)*sec(theta);
	psidot = q*sinf(phi)*(1/cosf(theta)) + r*cosf(phi)*(1/cosf(theta));
        
    pdot = c1*p*q-c2*q*r + c3*ell+c4*n;
    qdot = c5*p*r-c6*(p*p-r*r) + m/Iy;
    rdot = c7*p*q-c1*q*r + c4*ell+c8*n;

	struct state_rates body_rates =
	{
		pndot,
		pedot,
		pddot,
		udot,
		vdot,
		wdot,
		phidot,
		thetadot,
		psidot,
		pdot,
		qdot,
		rdot
	};

	return body_rates;
}

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
