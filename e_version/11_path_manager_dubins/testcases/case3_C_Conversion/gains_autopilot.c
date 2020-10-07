
#include <stdio.h>
#include "math_util.h"
#include "functions_vtol.h"
#include "parameters_vtol.h"

struct ap_gains gains_autopilot(struct states x_trim, struct trim_out y_trim, struct actuators u_trim, struct trans_funcs* tf)
{

struct ap_gains AP;

float Va_trim = sqrtf(x_trim.u*x_trim.u + x_trim.v*x_trim.v  + x_trim.w*x_trim.w);
//_________________________________
//	Set Autopilot Design Parameters
//_________________________________

	AP.tau = 0.05;

    //ROLL LOOP
    AP.delta_a_max = 30*PI/180;
    AP.e_phi_max   = 15*PI/180; 
    AP.zeta_phi     = .85;
    
    //COURSE LOOP
    AP.zeta_course     = 1.85;
    
    //SIDESLIP LOOP
    AP.delta_r_max = 30*PI/180;
    AP.e_beta_max  = 15*PI/180;
    AP.zeta_sideslip = 0.8;

    //SIDESLIP LOOP 
   /* Yv = A_lat(1,1);
	Yr = A_lat(1,3);
	Y_delta_r = B_lat(1,2);
	Nv = A_lat(3,1);
	Nr = A_lat(3,3);
	N_delta_r = B_lat(3,2);*/
    
    //PITCH LOOP
    AP.delta_e_max = 30*PI/180;
    AP.e_theta_max = 10*PI/180;
    AP.zeta_theta = .60;
    
    //ALTITUDE FROM PITCH LOOP
    AP.W_h = 43;
    AP.zeta_h = 1.8;
    
    //AIRSPEED FROM PITCH LOOP
    AP.W_V2 = 13;
    AP.zeta_V2 = 1.9;
    
    //AIRSPEED FROM THROTTLE LOOP
    AP.zeta_V = 2;
    AP.omega_n_V = 0.8;

//_______________________________
//	Compute Autopilot Gains
//_______________________________

//ROLL LOOP
    AP.roll_kp = (AP.delta_a_max/AP.e_phi_max)*sign(tf->b3);
	AP.omega_n_phi = sqrtf(fabsf(tf->b3)*(AP.delta_a_max/AP.e_phi_max));
	AP.roll_kd = (2*AP.zeta_phi*AP.omega_n_phi-tf->a2)/(tf->b3);

//SIDESLIP LOOP
	AP.sideslip_kp = (AP.delta_r_max / AP.e_beta_max)*sign((tf+7)->b3);
	AP.sideslip_ki = (1/(tf+7)->b3) * powf(((tf+7)->a3 + (tf+7)->b3*AP.sideslip_kp)/(2*AP.zeta_sideslip), 2);
    AP.d_beta = (1/vtol.Va0)*(x_trim.p*x_trim.w - x_trim.r*x_trim.u + vtol.g*cosf(x_trim.theta)*sinf(x_trim.phi)) + (0.5*vtol.rho*vtol.Va0*vtol.S_wing/vtol.m)*(vtol.CY0 + vtol.CYp*(vtol.b/(2*vtol.Va0))*x_trim.p + vtol.CYr*(vtol.b/(2*vtol.Va0))*x_trim.r + vtol.CYdelta_a*u_trim.delta_a);

//YAW DAMPER
   float Crp = vtol.gamma4*vtol.Clp + vtol.gamma8*vtol.Cnp;
   float Crr = vtol.gamma4*vtol.Clr + vtol.gamma8*vtol.Cnr;
   float Cr0 = vtol.gamma4*vtol.Cl0 + vtol.gamma8*vtol.Cn0;
   float Crbeta = vtol.gamma4*vtol.Clbeta + vtol.gamma8*vtol.Cnbeta;
   float Crdelta_a = vtol.gamma4*vtol.Cldelta_a + vtol.gamma8*vtol.Cndelta_a;
   float Crdelta_r = vtol.gamma4*vtol.Cldelta_r + vtol.gamma8*vtol.Cndelta_r;

   float Yv = (vtol.rho*vtol.S_wing*vtol.b*x_trim.v/(4*vtol.m*Va_trim))*(vtol.CYp*x_trim.p+vtol.CYr*x_trim.r) + (vtol.rho*vtol.S_wing*x_trim.v/vtol.m)*(vtol.CY0+vtol.CYbeta*y_trim.beta+vtol.CYdelta_a*u_trim.delta_a+vtol.    CYdelta_r*u_trim.delta_r) + (vtol.rho*vtol.S_wing*vtol.CYbeta/(2*vtol.m))*sqrtf(powf(x_trim.u,2)+powf(x_trim.w,2));
   float Nr = -vtol.gamma1*x_trim.q + (vtol.rho*Va_trim*vtol.S_wing*powf(vtol.b,2)/4)*Crr;
   float Yr = -x_trim.u + (vtol.rho*Va_trim*vtol.S_wing*vtol.b/(4*vtol.m))*vtol.CYr;
   float Nv = (vtol.rho*vtol.S_wing*powf(vtol.b,2)*x_trim.v/(4*Va_trim))*(Crp*x_trim.p+Crr*x_trim.r) + (vtol.rho*vtol.S_wing*vtol.b*x_trim.v)*(Cr0+Crbeta*y_trim.beta+Crdelta_a*u_trim.delta_a+Crdelta_r*u_trim.delta_r) + (vtol.rho*vtol.S_wing*vtol.b*Crbeta/2)*sqrtf(powf(x_trim.u,2)+powf(x_trim.w,2));
   float N_delta_r = (vtol.rho*powf(Va_trim,2)*vtol.S_wing*vtol.b/2)*Crdelta_r;
   float Y_delta_r = (vtol.rho*powf(Va_trim,2)*vtol.S_wing/(2*vtol.m))*vtol.CYdelta_r;

    AP.omega_n_dr = sqrtf(Yv*Nr - Yr*Nv); // dutch-roll frequency
	AP.p_wo = AP.omega_n_dr/10; // pole of the washout filter
	AP.yaw_damper_tau_r = (1/AP.p_wo);

	float value1 = (Nr*N_delta_r + Y_delta_r*Nv)/powf(N_delta_r,2);

	// AP.yaw_damper_kp below denotes Kr
	AP.yaw_damper_kp = - value1 + sqrtf(powf(value1,2) - (Yv*Yv + Nr*Nr + 2*Yr*Nv) / powf(N_delta_r,2));

//COURSE LOOP
	AP.omega_n_chi = AP.omega_n_dr/30;
	AP.course_kp = 2 * AP.zeta_course * AP.omega_n_chi * Va_trim/vtol.g;
	AP.course_ki = powf(AP.omega_n_chi,2) * Va_trim/vtol.g;
	AP.d_chi = tanf(x_trim.phi) - x_trim.phi;

//PITCH LOOP
	AP.pitch_kp = (AP.delta_e_max/AP.e_theta_max)*sign((tf+2)->b3);
	AP.omega_n_theta = sqrtf((tf+2)->a3 + fabsf((tf+2)->b3)*(AP.delta_e_max/AP.e_theta_max));
	AP.pitch_kd = (2*AP.zeta_theta*AP.omega_n_theta - (tf+2)->a2)/((tf+2)->b3);
	AP.K_theta_DC = AP.pitch_kp * (tf+2)->b3/((tf+2)->a3 + AP.pitch_kp * (tf+2)->b3);

//ALTITUDE FROM PITCH LOOP
    AP.omega_n_h = (1/AP.W_h)*AP.omega_n_theta;
    AP.altitude_ki = powf(AP.omega_n_h,2) / (AP.K_theta_DC * vtol.Va0);
    AP.altitude_kp = (2 * AP.zeta_h * AP.omega_n_h)/(AP.K_theta_DC * vtol.Va0);
    AP.d_h = (x_trim.u*sinf(x_trim.theta)-vtol.Va0*x_trim.theta) - x_trim.v*sinf(x_trim.phi)*cosf(x_trim.theta) - x_trim.w*cosf(x_trim.phi)*cosf(x_trim.theta);

//AIRSPEED FROM PITCH LOOP
    AP.omega_n_V2 = (1/AP.W_V2)*AP.omega_n_theta;
    AP.kp_V2 = ((tf+6)->a3 - 2 * AP.zeta_V2 * AP.omega_n_V2)/(AP.K_theta_DC * vtol.g);
    AP.ki_V2 = - powf(AP.omega_n_V2,2) / (AP.K_theta_DC * vtol.g);

//AIRSPEED FROM THROTTLE
    AP.ki_V = powf(AP.omega_n_V,2) / (tf+5)->b3;
    AP.kp_V = (2 * AP.zeta_V * AP.omega_n_V - (tf+5)->a3) / (tf+5)->b3;

return AP;

}
