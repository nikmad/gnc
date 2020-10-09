/*
Author: Nikhil Madduri
Created: 31/Jul/2020
Modified: 01/Aug/2020 till 08/Oct/2020
*/

#include "math_util.h"
#include "functions_vtol.h"
#include "parameters_vtol.h"

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
	
	K1_struct = sixDOF(y, uu);

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
	    yt[i] = y[i] + (h/2) * K1[i];
	}
	//_______________________________________________________

	K2_struct = sixDOF(yt,uu);

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
		yt[i] = y[i] + (h/2) * K2[i];
	}

	//_______________________________________________________

	K3_struct = sixDOF(yt,uu);

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

	K4_struct = sixDOF(yt,uu);

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
	    y[i] = y[i] + (h/6)*( K1[i] + 2.0*( K2[i] + K3[i] ) + K4[i] );
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

	float theta1 = (float)modulo_(absolut_(theta),2*PI);
	float theta2 = (float)floor(absolut_(theta)/(2*PI))*(2*PI);

	if ((floor(theta1/(PI/2))==0) && (PI/2-theta1 < sensitive90zone))
		{theta = sign_(theta)*(PI/2 - sensitive90zone + theta2);}
	else if ((floor(theta1/(PI/2))==1) &&   (theta1-PI/2 < sensitive90zone))
		{theta = sign_(theta)*(PI/2 + sensitive90zone + theta2);}
	else if ((floor(theta1/(PI/2))==2) && (3*PI/2-theta1 < sensitive90zone))
		{theta = sign_(theta)*(3*PI/2 - sensitive90zone + theta2);}
	else if ((floor(theta1/(PI/2))==3) && (theta1-3*PI/2 < sensitive90zone))
		{theta = sign_(theta)*(3*PI/2 + sensitive90zone + theta2);}

	phidot = p + q*sinf(phi)*tanf(theta) + r*cosf(phi)*tanf(theta);
	thetadot = q*cosf(phi)-r*sinf(phi);
	psidot = q*sinf(phi)*sec(theta) + r*cosf(phi)*sec(theta);
        
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

