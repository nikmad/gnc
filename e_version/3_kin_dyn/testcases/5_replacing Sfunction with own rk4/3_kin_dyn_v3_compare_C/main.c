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

struct force_n_moments forces_moments(struct states, struct actuators, struct wnd);
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
	struct force_n_moments fm_in = {0.5023,0,0, 0,0,0, 0,0,0, 0,0,0};
	struct actuators delta = {0.047,0,0,0};
	struct wnd _wind = {0.001,0.001,0.001,0.001,0.001,0.001};

	int i;
	float t = 0.0;

	float Va = states_in.u; //DUMMY VALUE ONLY FOR TESTING

	FILE *fptr;
	fptr = fopen("nikstates.txt", "w+");

	fprintf(fptr, "%3.3f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", t, states_in.pn, states_in.pe, states_in.pd, states_in.u, states_in.v, states_in.w, states_in.phi, states_in.theta, states_in.psi, states_in.p, states_in.q, states_in.r, Va, 0.0,0.0,0.0,0.0,0.0,0.0,0.0);

	for(i=1; i<5001; i++)
	{
		t = i*SIM.rk4_stepsize;
		//fm_in = forces_moments(states_in, delta, _wind);	
	    states_out = vtol_dynamics(states_in, fm_in);
		//               1     2    3    4    5    6    7    8    9     10  11   12   13   14    15    16   17  18   19   20   21
		//               t     pn   pe   pd   u    v    w   phi  theta psi  p    q    r    Va  alpha beta chi delE delA delR delT
		fprintf(fptr, "%3.3f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", t, states_out.pn, states_out.pe, states_out.pd, states_out.u, states_out.v, states_out.w, states_out.phi, states_out.theta, states_out.psi, states_out.p, states_out.q, states_out.r, Va, 0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		states_in = states_out;
   		//fputs("This is testing for fputs...\n", fptr);
	}

	fclose(fptr);
	
	//printf("Fn1: %f and %f\n", vtol.Cnp, states_out.q);
	
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
	    yt[i] = y[i] + (h/2) * K1[i];
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
		yt[i] = y[i] + (h/2) * K2[i];
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
	//printf("Test fx = %f\n",fx);

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

    //printf("value of theta = %f\n", theta);

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

//__________________________________________________________

struct force_n_moments forces_moments(struct states states_in, struct actuators delta, struct wnd wind)
{
	struct force_n_moments fm_out;

	float R_v_v1[3][3];
	R_v_v1[0][0] = cosf(states_in.psi);
	R_v_v1[0][1] = sinf(states_in.psi);
	R_v_v1[0][2] = 0;
	
	R_v_v1[1][0] = -sinf(states_in.psi);
	R_v_v1[1][1] = cosf(states_in.psi);
	R_v_v1[1][2] = 0;
	
	R_v_v1[2][0] = 0;
	R_v_v1[2][1] = 0;
	R_v_v1[2][2] = 1;

	float R_v1_v2[3][3];
	R_v1_v2[0][0] = cosf(states_in.theta);
	R_v1_v2[0][1] = 0;
	R_v1_v2[0][2] = -sinf(states_in.theta);
	
	R_v1_v2[1][0] = 0;
	R_v1_v2[1][1] = 1;
	R_v1_v2[1][2] = 0;
	
	R_v1_v2[2][0] = sinf(states_in.theta);
	R_v1_v2[2][1] = 0;
	R_v1_v2[2][2] = cosf(states_in.theta);

	float R_v2_b[3][3];
	R_v2_b[0][0] = 1;
	R_v2_b[0][1] = 0;
	R_v2_b[0][2] = 0;
	
	R_v2_b[1][0] = 0;
	R_v2_b[1][1] = cosf(states_in.phi);
	R_v2_b[1][2] = sinf(states_in.phi);
	
	R_v2_b[2][0] = 0;
	R_v2_b[2][1] = -sinf(states_in.phi);
	R_v2_b[2][2] = cosf(states_in.phi);

	float temp3X3_1[3][3];
	float R_v_b[3][3];

	//Creating the rotation matrix
	MatrixMultiply(R_v1_v2,3,3,R_v_v1,3,3,temp3X3_1);
	MatrixMultiply(R_v2_b,3,3,temp3X3_1,3,3,R_v_b);

	float temp3x1_1[3][1], temp3x1_2[3][1];
	temp3x1_1[0][0] = wind.w_ns;
	temp3x1_1[1][0] = wind.w_es;
	temp3x1_1[2][0] = wind.w_ds;

	//Converting steady wind from NED to body frame
	MatrixMultiply(R_v_b,3,3,temp3x1_1,3,1,temp3x1_2);

	//Total wind vecotr in body-frame: adding the steady components and wind gust components in body frame
	float V_w[3][1];
	V_w[0][0] = temp3x1_2[0][0] + wind.u_wg;
	V_w[1][0] = temp3x1_2[1][0] + wind.v_wg;
	V_w[2][0] = temp3x1_2[2][0] + wind.w_wg;

	//Body-frame components of the airspeed vector
	float u_r, v_r, w_r;
	u_r = states_in.u - V_w[0][0];
	v_r = states_in.v - V_w[1][0];
	w_r = states_in.w - V_w[2][0];

	//compute air data
	fm_out.Va = sqrt(u_r*u_r+v_r*v_r+w_r*w_r);
	fm_out.alpha = atan(w_r/u_r);
	fm_out.beta = asin(v_r/fm_out.Va);

	//Total wind vector in NED frame
	float V_v[3][1];
	transposed3x3(R_v_b);
	float R_b_v[3][3];

	//R_b_v[][] = R_v_b[][];

	int i, j;
	for(i=0; i<3; i++)
	for(j=0; j<3; j++)
	{
		R_b_v[i][j] = R_v_b[i][j];
	}

	MatrixMultiply(R_b_v,3,3,V_w,3,1,V_v);

	fm_out.w_n = V_v[0][0];
	fm_out.w_e = V_v[1][0];
	fm_out.w_d = V_v[2][0];

	// compute external forces and torques on aircraft

    //====================================================
    // Gravity force
    //====================================================
	float fg_x, fg_y, fg_z;

	fg_x = -vtol.m * vtol.g * sinf(states_in.theta);
	fg_y = vtol.m * vtol.g * cosf(states_in.theta) * sinf(states_in.phi);
	fg_z = vtol.m * vtol.g * cosf(states_in.theta) * cosf(states_in.phi);

	//====================================================
    // Aerodynamic forces
    //====================================================

    float CDalpha = vtol.CDp  + (powf(vtol.CL0 + vtol.CLalpha*abs(fm_out.alpha),2))/(pi*vtol.e*vtol.AR);
   	float sigma_alpha = (1 + exp(-vtol.M*(fm_out.alpha-vtol.alpha0)) + exp(vtol.M*(fm_out.alpha+vtol.alpha0)))/((1+exp(-vtol.M*(fm_out.alpha-vtol.alpha0)))*(1+exp(vtol.M*(fm_out.alpha+vtol.alpha0))));
   	float CLalpha = (1-sigma_alpha)*(vtol.CL0 + vtol.CLalpha*fm_out.alpha) + sigma_alpha*(2*sign(fm_out.alpha)*powf(sinf(fm_out.alpha),2)*cosf(fm_out.alpha));
    
    float CXalpha = -CDalpha*cosf(fm_out.alpha) + CLalpha*sinf(fm_out.alpha);
    float CXq_alpha = -vtol.CDq*cosf(fm_out.alpha) + vtol.CLq*sinf(fm_out.alpha);
    float CXdelta_e_alpha = -vtol.CDdelta_e*cosf(fm_out.alpha) + vtol.CLdelta_e*sinf(fm_out.alpha);
    float CZalpha = -CDalpha*sinf(fm_out.alpha) - CLalpha*cosf(fm_out.alpha);
    float CZq_alpha = -vtol.CDq*sinf(fm_out.alpha) - vtol.CLq*cosf(fm_out.alpha);
    float CZdelta_e_alpha = -vtol.CDdelta_e*sinf(fm_out.alpha) - vtol.CLdelta_e*cosf(fm_out.alpha);

    float fa_x = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*(CXalpha + CXq_alpha*(vtol.c/(2*fm_out.Va))*states_in.q + CXdelta_e_alpha*abs(delta.delta_e)); 
    float fa_y = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*(vtol.CY0 + vtol.CYbeta*fm_out.beta + vtol.CYp*(vtol.b/(2*fm_out.Va))*states_in.p + vtol.CYr*(vtol.b/(2*fm_out.Va))*states_in.r + vtol.CYdelta_a*delta.delta_a + vtol.CYdelta_r*delta.delta_r);
    float fa_z = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*(CZalpha + CZq_alpha*(vtol.c/(2*fm_out.Va))*states_in.q + CZdelta_e_alpha*delta.delta_e);

    //====================================================
    // Engines/Motors(propulsion system) forces
    //====================================================
    float C_prop = 1.0;
    float k_motor = 80;

    float Vin = vtol.Vmax * delta.delta_t;

    //parameters of quadratic equation solution
    float a_omega = vtol.rho*powf(vtol.D_prop,5)*vtol.CQ0/powf(2*pi,2);
    float b_omega = vtol.rho*powf(vtol.D_prop,4)*vtol.CQ1*fm_out.Va/(2*pi) + powf(vtol.KQ,2)/vtol.Rmotor;
    float c_omega = vtol.rho*powf(vtol.D_prop,3)*vtol.CQ2*powf(fm_out.Va,2) - vtol.KQ * Vin/vtol.Rmotor + vtol.KQ * vtol.i0;
    
    float Omega_p = (-b_omega + sqrt(powf(b_omega,2)-4*a_omega*c_omega))/(2*a_omega);
    float Tp = vtol.rho*powf(vtol.D_prop,4)*vtol.CT0*powf(Omega_p,2)/(4*pi*pi)+ vtol.rho*powf(vtol.D_prop,3)*vtol.CT1*fm_out.Va*Omega_p/(2*pi) + vtol.rho*powf(vtol.D_prop,2)*vtol.CT2*powf(fm_out.Va,2);
    float Qp = vtol.rho*powf(vtol.D_prop,5)*vtol.CQ0*powf(Omega_p,2)/(4*pi*pi)+ vtol.rho*powf(vtol.D_prop,4)*vtol.CQ1*fm_out.Va*Omega_p/(2*pi) + vtol.rho*powf(vtol.D_prop,3)*vtol.CQ2*powf(fm_out.Va,2);

    float fp_x = Tp; 
    float fp_y = 0.0;
    float fp_z = 0.0;

    //====================================================
    // Total Force in Body frame
    //====================================================
    fm_out.fx =  fg_x+fa_x+fp_x; // fx
    fm_out.fy =  fg_y+fa_y+fp_y; // fy
    fm_out.fz =  fg_z+fa_z+fp_z; // fz

    //====================================================
    // Aerodynamic Moments
    //====================================================
    float ell_a = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*vtol.b*(vtol.Cl0 + vtol.Clbeta*fm_out.beta + vtol.Clp*(vtol.b/(2*fm_out.Va))*states_in.p + vtol.Clr*(vtol.b/(2*fm_out.Va))*states_in.r + vtol.Cldelta_a*delta.delta_a + vtol.Cldelta_r*delta.delta_r);
    float m_a = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*vtol.c*(vtol.Cm0 + vtol.Cmalpha*fm_out.alpha + vtol.Cmq*(vtol.c/(2*fm_out.Va))*states_in.q + vtol.Cmdelta_e*delta.delta_e);
    float n_a = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*vtol.b*(vtol.Cn0 + vtol.Cnbeta*fm_out.beta + vtol.Cnp*(vtol.b/(2*fm_out.Va))*states_in.p + vtol.Cnr*(vtol.b/(2*fm_out.Va))*states_in.r + vtol.Cndelta_a*delta.delta_a + vtol.Cndelta_r*delta.delta_r);

    //====================================================
    // Engines/motors(propulsion system) Moments
    //====================================================
    float k_T_p = 0;
    float k_omega = 0;

    float ell_p = Qp;
    float m_p = 0;
    float n_p = 0;

    //====================================================
    // Total Moments in Body frame
    //====================================================

     fm_out.l = ell_a+ell_p;
     fm_out.m = m_a+m_p;   
     fm_out.n = n_a+n_p;

     return fm_out;
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
