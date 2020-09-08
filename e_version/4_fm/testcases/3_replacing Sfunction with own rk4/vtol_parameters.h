/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/

// so that constants are not read from multiple files
#include <math.h>

#ifndef MYLIB_CONSTANTS_H

//#define pi acos(-1)
#define pi 3.141592653589793238

//_______________________________________________________
// defining structures 

struct vt{
 float Va0;
 float gamma;
 float R;
 float Ts;
 
 //physical parameters
 float g;
 float m;
 
 float Jx;
 float Jy;
 float Jz;
 float Jxz;
 
 float S_wing;
 float S_prop;
 float b;
 float c;
 float e;
 float AR;
 
 float gamma0;
 float gamma1;
 float gamma2;
 float gamma3;
 float gamma4;
 float gamma5;
 float gamma6;
 float gamma7;
 float gamma8;
 
 //aerodynamic coefficients
 float CL0;
 float CD0;
 float Cm0;
 
 float CLalpha;
 float CDalpha;
 float Cmalpha;
 
 float CLq;
 float CDq;
 float Cmq;
 
 float CDp;
 
 float CLdelta_e;
 float CDdelta_e;
 
 float CY0;
 float Cl0;
 float Cn0;
 float CYbeta;
 float Clbeta;
 float Cnbeta;
 float CYp;	
 float Clp;
 float Cnp;
 float CYr;
 float Clr;
 float Cnr;
 float CYdelta_a;
 
 float Cldelta_a;
 float Cndelta_r;
 float Cmdelta_e;
 
 float Cndelta_a;
 float CYdelta_r;
 float Cldelta_r;
 
 float M;
 float alpha0;
 float epsilon;
 
 //propulsion parameters
 float D_prop;
 float KV;
 float KQ;
 float Rmotor;
 float i0;
 float ncells;	
 float Vmax;
 float CQ0;
 float CQ1;
 float CQ2;
 float CT0;	
 float CT1;
 float CT2;
 
 //atmospheric conditions
 float rho;
};

struct states{
	float pn;
	float pe;
	float pd;
	float u;
	float v;
	float w;
	float phi;
	float theta;
	float psi;
	float p;
	float q;
	float r;
};

struct state_rates{
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
};

struct force_n_moments{
	float fx;
	float fy;
	float fz;
	
	float l;
	float m;
	float n;
	
	float Va;
	float alpha;
	float beta;

	//Total wind in NED (inertial)
	float w_n;
	float w_e;
	float w_d;
};

struct actuators{
	float delta_e; 
	float delta_a; 
	float delta_r; 
	float delta_t;
};

struct wnd{
	float w_ns; //steady wind - North   
	float w_es; //steady wind - East   
	float w_ds; //steady wind - Down   
	float u_wg; //gust along body x-axis
	float v_wg; //gust along body y-axis
	float w_wg; //gust along body z-axis
};

struct simulation_params{
	float ts_simulation;
	float ts_control;
	float ts_plotting;
	float ts_video;
	float rk4_stepsize;
};

//------------------------
const struct vt vtol = 
{
	10.0,	//float Va0;
	15*pi/180,	//float gamma;
	INFINITY, 	//float R;
	0.01,	//float Ts;
	//
	////physical parameters
	9.81,	//float g;
	13.5,	//float m = mass;
	//
	0.8244,	//float Jx;
	1.135,	//float Jy;
	1.759,	//float Jz;
	0.1204, //float Jxz;
	//
	0.55,	//float S_wing;
	0.2027,	//float S_prop;
	2.90,	//float b;
	0.19,	//float c;
	0.9,	//float e;
	15.2909,	//float AR;

	//
	1.4356,	//float gamma0;
	0.1215,	//float gamma1;
	0.7747,	//float gamma2;
	1.2253,	//float gamma3;
	0.0839,	//float gamma4;
	0.8234,	//float gamma5;
	0.1061,	//float gamma6;
	-0.1683,//float gamma7;
	0.5742,	//float gamma8;
	//
	////aerodynamic coefficients
	0.28,	//float CL0;
	0.03,	//float CD0;
	-0.02338,	//float Cm0;
	//
	3.45,	//float CLalpha;
	0.3,	//float CDalpha;
	-0.38,	//float Cmalpha;
	//
	0,	//float CLq;
	0.0,	//float CDq;
	-3.6,	//float Cmq;
	//
	0.0437,	//float CDp;
	//
	-0.36,	//float CLdelta_e;
	0.0,	//float CDdelta_e;
	//
	0.0,	//float CY0;
	0.0,	//float Cl0;
	0.0,	//float Cn0;
	-0.98,	//float CYbeta;
	-0.12,	//float Clbeta;
	0.25,	//float Cnbeta;
	0.0,	//float CYp;	
	-0.26,	//float Clp;
	0.022,	//float Cnp;
	0.0,	//float CYr;
	0.14,	//float Clr;
	-0.35,	//float Cnr;
	0.0,	//float CYdelta_a;
	//
	0.08,	//float Cldelta_a;
	-0.032,	//float Cndelta_r;
	-0.5,	//float Cmdelta_e;
	//
	0.06,	//float Cndelta_a;
	-0.17, 	//float CYdelta_r;
	0.105,	//float Cldelta_r;
	//
	50, 	//float M;
	0.4712,	//float alpha0;
	0.16,	//float epsilon;
	//
	////propulsion parameters
	0.508,	//float D_prop;
	145.0,	//float KV;
	0.0659,	//float KQ;
	0.042,	//float Rmotor;
	1.5,	//float i0;
	12.0,	//float ncells;	
	44.4,	//float Vmax;
	0.005230,	//float CQ0;
	0.004970,	//float CQ1;
	-0.01664,	//float CQ2;
	0.09357,	//float CT0;	
	-0.06044,	//float CT1;
	-0.1079,	//float CT2;
	//
	////atmospheric conditions
	1.2682	//float rho;	
}; 

const struct simulation_params SIM = 
{
	0.02,	//float ts_simulation;
	0.02,	//float ts_control;
	0.1,	//float ts_plotting;
	0.1,	//float ts_video;
	0.01	//float rk4_stepsize;
};
//---------------------

#define MYLIB_CONSTANTS_H
#endif


