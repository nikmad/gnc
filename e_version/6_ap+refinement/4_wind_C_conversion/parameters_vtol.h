/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/

// so that constants are not read from multiple files
#ifndef  EXTERN
#define  EXTERN  extern
#endif

#include <math.h>

#ifndef MYLIB_CONSTANTS_H
#include "math_util.h"

//#define pi acos(-1)
//#define pi 3.141592653589793238

//_______________________________________________________
// defining structures 

EXTERN struct vt{
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

EXTERN struct states{
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

EXTERN struct force_n_moments{
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

EXTERN struct actuators{
	float delta_e; 
	float delta_a; 
	float delta_r; 
	float delta_t;
};

EXTERN struct trim_out{
	float Va; 
	float alpha; 
	float beta; 
};

EXTERN struct wnd{
	float w_ns; //steady wind - North   
	float w_es; //steady wind - East   
	float w_ds; //steady wind - Down   
	float u_wg; //gust along body x-axis
	float v_wg; //gust along body y-axis
	float w_wg; //gust along body z-axis
};

EXTERN struct simulation_params{
	float ts_simulation;
	float ts_control;
	float ts_plotting;
	float ts_video;
	float rk4_stepsize;
};

EXTERN const struct vt vtol;
EXTERN const struct simulation_params SIM;

#define MYLIB_CONSTANTS_H
#endif


