/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/

// so that constants are not read from multiple files
#ifndef MYLIB_CONSTANTS_H

//_______________________________________________________
// defining structures 

struct vtol{
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

	float gamma;
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

struct force_n_moments{
	float fx;
	float fy;
	float fz;
	float l;
	float m;
	float n;
};

struct simulation_params{
	float ts_simulation;
	float ts_control;
	float ts_plotting;
	float ts_video;
	float rk4_stepsize;
};

//------------------------
struct vt{
	float Va0;
	float biscuit;
};

const struct vt vtol = 
{
	35.0, //Va0
	20.0		
}; 

//---------------------
#define MYLIB_CONSTANTS_H
#endif


