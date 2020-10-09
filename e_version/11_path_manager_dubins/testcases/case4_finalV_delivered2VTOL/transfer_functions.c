/*
Author: Nikhil Madduri
Created: 31/Jul/2020
Modified: 01/Aug/2020 till 08/Oct/2020
*/

#include <stdio.h>
#include "math_util.h"
#include "functions_vtol.h"
#include "parameters_vtol.h"

struct trans_funcs* transfer_functions(struct states x_trim, struct trim_out y_trim, struct actuators u_trim)
{
	struct trans_funcs* tf;
	tf = (struct trans_funcs*)malloc(8*sizeof(struct trans_funcs));

	float Va_trim = sqrtf(x_trim.u*x_trim.u + x_trim.v*x_trim.v  + x_trim.w*x_trim.w);

	float theta_trim = x_trim.theta;
	float alpha_trim  = y_trim.alpha;
	float delta_e_trim = u_trim.delta_e;
	float delta_t_trim = u_trim.delta_t;

	float Cp0       = vtol.gamma3 * vtol.Cl0        + vtol.gamma4 * vtol.Cn0;
	float Cpbeta    = vtol.gamma3 * vtol.Clbeta     + vtol.gamma4 * vtol.Cnbeta;
	float Cpp       = vtol.gamma3 * vtol.Clp        + vtol.gamma4 * vtol.Cnp;
	float Cpr       = vtol.gamma3 * vtol.Clr        + vtol.gamma4 * vtol.Cnr;
	float Cpdelta_a = vtol.gamma3 * vtol.Cldelta_a  + vtol.gamma4 * vtol.Cndelta_a;
	float Cpdelta_r = vtol.gamma3 * vtol.Cldelta_r  + vtol.gamma4 * vtol.Cndelta_r;

	float value1 = 0.5 * vtol.rho * powf(Va_trim,2) * vtol.S_wing * vtol.b;
	float value2 = 0.5 * vtol.rho * powf(Va_trim,2) * vtol.S_wing * vtol.c / vtol.Jy;
	float value3 = 0.5 * vtol.rho * Va_trim * vtol.S_wing / vtol.m;

	float a_phi1 =  -value1 * Cpp * vtol.b/(2*Va_trim);
	float a_phi2 =   value1 * Cpdelta_a;

	float a_theta1 = -value2 * vtol.Cmq * vtol.c /(2*Va_trim);
	float a_theta2 = -value2 * vtol.Cmalpha;
	float a_theta3 =  value2 * vtol.Cmdelta_e;

	float Vin = vtol.Vmax * delta_t_trim;

//parameters of quadratic equation solution

	// parameters defined to merely simplify the actual expression of Omega_p
	float a_omega = vtol.rho*powf(vtol.D_prop,5)*vtol.CQ0/powf(2*PI,2);
	float b_omega = vtol.rho*powf(vtol.D_prop,4)*vtol.CQ1*Va_trim/(2*PI) + powf(vtol.KQ,2)/vtol.Rmotor;
	float c_omega = vtol.rho*powf(vtol.D_prop,3)*vtol.CQ2*powf(Va_trim,2) - vtol.KQ * Vin/vtol.Rmotor + vtol.KQ * vtol.i0;

	float Omega_p = (-b_omega + sqrtf(powf(b_omega,2)-4*a_omega*c_omega))/(2*a_omega);

	// parameters defined to merely simplify the actual expression of dTpVa and dTpDelta_t
	float vk1 = vtol.rho*powf(vtol.D_prop,4)*vtol.CT0/powf(2*PI,2);
	float vk2 = vtol.rho*powf(vtol.D_prop,3)*vtol.CT1/(2*PI);
	float vk3 = vtol.rho*powf(vtol.D_prop,2)*vtol.CT2;
	float vk4 = vtol.rho*powf(vtol.D_prop,4)*vtol.CQ1/(2*PI);
	float vk5 = powf(vtol.KQ,2) / vtol.Rmotor;
	float vk6 = vtol.rho*powf(vtol.D_prop,3) * vtol.CQ2;
	float vk7 = -vtol.Vmax * (vtol.KQ / vtol.Rmotor);
	float vk8 = vtol.KQ * vtol.i0;
	float vk9 = powf(vk4 * Va_trim + vk5, 2) - 4*a_omega*(vk6 * powf(Va_trim,2) + vk7 * delta_t_trim + vk8); // = b^2 - 4*a*c
	float vk10 = 2*(vk4*Va_trim + vk5)*vk4 - 4*a_omega*(2*vk6*Va_trim); // vk10 = d(vk9)/d(Va)

	float dOmegaVa = (1/(2*a_omega))*(-vk4 + 0.5 * (1/sqrtf(vk9)) * vk10); // = d(Omega_p)/d(Va)

	float dTpVa       = 2 * vk1 * Omega_p * dOmegaVa + vk2*(Va_trim * dOmegaVa + Omega_p) + 2 * vk3 * Va_trim; // = d(T_p)/d(Va)
	float dTpDelta_t  = (-vk7/sqrtf(vk9)) * (2*vk1*Omega_p + vk2*Va_trim); // = d(T_p)/d(delta_t)

	float a_V1 = (1/vtol.m) * (vtol.rho * Va_trim * vtol.S_wing *(vtol.CD0 + vtol.CDalpha*alpha_trim + vtol.CDdelta_e * delta_e_trim) - dTpVa); 
	float a_V2 = (1/vtol.m) * dTpDelta_t;
	float a_V3 = vtol.g * cosf(theta_trim - alpha_trim); 

	float a_beta1 = -value3 * vtol.CYbeta;
	float a_beta2 =  value3 * vtol.CYdelta_r;
//                                     b0 	b1  b1    b3            a0  a1   a2         a3
	struct trans_funcs tf_phiDelA 	= {0,	0,	0,	a_phi2,			0,	1,	a_phi1,		0};
	struct trans_funcs tf_chiPhi 	= {0,	0,	0,	vtol.g/Va_trim,	0,	0,	1,			0};
	struct trans_funcs tf_thetaDelE = {0,	0,	0,	a_theta3,		0,	1,	a_theta1,	a_theta2};
	struct trans_funcs tf_hTheta 	= {0,	0,	0,	Va_trim,		0,	0,	1,			0};
	struct trans_funcs tf_hVa 		= {0,	0,	0,	theta_trim,		0,	0,	1,			0};
	struct trans_funcs tf_VaDelT 	= {0,	0,	0,	a_V2,			0,	0,	1,			a_V1};
	struct trans_funcs tf_VaTheta 	= {0,	0,	0,	-a_V3,			0,	0,	1,			a_V1};
	struct trans_funcs tf_betaDelR 	= {0,	0,	0,	a_beta2,		0,	0,	1,			a_beta1};

	*tf 	= tf_phiDelA;
	*(tf+1)	= tf_chiPhi;
	*(tf+2)	= tf_thetaDelE;
	*(tf+3)	= tf_hTheta;
	*(tf+4)	= tf_hVa;
	*(tf+5)	= tf_VaDelT;
	*(tf+6)	= tf_VaTheta;
	*(tf+7)	= tf_betaDelR;

	return tf;

	free(tf);
}
