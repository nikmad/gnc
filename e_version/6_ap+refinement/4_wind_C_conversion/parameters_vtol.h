/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/

// so that constants are not read from multiple files
#ifndef  EXTERN
#define  EXTERN  extern
#endif

#include <math.h>
#include "math_util.h"

#ifndef MYLIB_PARAMS_H

//#define pi acos(-1)
//#define pi 3.141592653589793238

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

struct trim_out{
	float Va; 
	float alpha; 
	float beta; 
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

struct trans_funcs{
	//numerator coefficients
	float b0;
	float b1;
	float b2;
	float b3;
	//denominator coefficients
	float a0;
	float a1;
	float a2;
	float a3;
};

struct ap_gains{
    
    //ROLL LOOP
    float tau;
    float delta_a_max;
    float e_phi_max;
    float zeta_phi;

    float roll_kp;  
    float omega_n_phi;  
    float roll_kd;
    
    //SIDESLIP LOOP
    float delta_r_max;
    float e_beta_max; 
    float zeta_sideslip;

    float sideslip_kp;
    float sideslip_ki;
    float d_beta;

    //YAW DAMPER
    float omega_n_dr;
    float p_wo;
    float yaw_damper_tau_r;
    float yaw_damper_kp;

    //COURSE LOOP
    float zeta_course;

    float omega_n_chi;
    float course_kp;
    float course_ki;
    float d_chi;
    
    //PITCH LOOP
    float delta_e_max;
    float e_theta_max;
    float zeta_theta; 

    float pitch_kp;
    float omega_n_theta;
    float pitch_kd;
    float K_theta_DC;

    //ALTITUDE FROM PITCH LOOP
    float W_h;
    float zeta_h;

    float omega_n_h;
    float altitude_ki;
    float altitude_kp;
    float d_h;

    //AIRSPEED FROM PITCH LOOP
    float W_V2;
    float zeta_V2;

    float omega_n_V2;
    float kp_V2;
    float ki_V2;

    //AIRSPEED FROM THROTTLE LOOP
    float zeta_V;
    float omega_n_V;

    float ki_V;
    float kp_V;

};

const struct vt vtol;
const struct simulation_params SIM;

//---------------------
#define MYLIB_PARAMS_H
#endif


