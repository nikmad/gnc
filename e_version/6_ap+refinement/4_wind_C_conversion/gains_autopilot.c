
#include <stdio.h>
#include "math_util.h"
#include "functions_vtol.h"
#include "parameters_vtol.h"

gains_autopilot()
{

struct ap_gains AP;

float Va_trim = sqrt(x_trim.u*x_trim.u + x_trim.v*x_trim.v  + x_trim.w*x_trim.w);
//_________________________________
//	Set Autopilot Design Parameters
//_________________________________

	AP.tau = 0.05;

    //ROLL LOOP
    AP.delta_a_max = 30*pi/180;
    AP.e_phi_max   = 15*pi/180; 
    AP.zeta_phi     = .85;
    
    //COURSE LOOP
    AP.zeta_course     = 1.85;
    
    //SIDESLIP LOOP
    AP.delta_r_max = 30*pi/180;
    AP.e_beta_max  = 15*pi/180;
    AP.zeta_sideslip = 0.8;

    //SIDESLIP LOOP 
    Yv = A_lat(1,1);
	Yr = A_lat(1,3);
	Y_delta_r = B_lat(1,2);
	Nv = A_lat(3,1);
	Nr = A_lat(3,3);
	N_delta_r = B_lat(3,2);
    
    //PITCH LOOP
    AP.delta_e_max = 30*pi/180;
    AP.e_theta_max = 10*pi/180;
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
	AP.omega_n_phi = sqrt(abs(tf->b3)*(AP.delta_a_max/AP.e_phi_max));
	AP.roll_kd = (2*AP.zeta_phi*AP.omega_n_phi-tf->a2)/(tf->b3);

//SIDESLIP LOOP
	AP.sideslip_kp = (AP.delta_r_max / AP.e_beta_max)*sign((tf+7)->b3);
	AP.sideslip_ki = (1/(tf+7)->b3) * powf(((tf+7)->a3 + (tf+7)->b3*AP.sideslip_kp)/(2*AP.zeta_sideslip), 2);

	AP.omega_n_dr = sqrt(Yv*Nr - Yr*Nv); // dutch-roll frequency
	AP.p_wo = AP.omega_n_dr/10; // pole of the washout filter
	AP.yaw_damper_tau_r = (1/AP.p_wo);

	value1 = (Nr*N_delta_r + Y_delta_r*Nv)/powf(N_delta_r,2);

	// AP.yaw_damper_kp below denotes Kr
	AP.yaw_damper_kp = - value1 + sqrt(powf(value1,2) - (Yv*Yv + Nr*Nr + 2*Yr*Nv) / powf(N_delta_r,2));

//COURSE LOOP
	AP.omega_n_chi = AP.omega_n_dr/30;
	AP.course_kp = 2 * AP.zeta_course * AP.omega_n_chi * Va_trim/vtol.g;
	AP.course_ki = powf(AP.omega_n_chi,2) * Va_trim/vtol.gravity;
	AP.d_chi = tanf(x_trim.phi) - x_trim.phi;

//PITCH LOOP
	AP.pitch_kp = (AP.delta_e_max/AP.e_theta_max)*sign((tf+2)->b3);
	AP.omega_n_theta = sqrt((tf+2)->a3 + abs((tf+2)->b3)*(AP.delta_e_max/AP.e_theta_max));
	AP.pitch_kd = (2*AP.zeta_theta*AP.omega_n_theta - (tf+2)->a2)/((tf+2)->b3);
	AP.K_theta_DC = AP.pitch_kp * (tf+2)->b3/((tf+2)->a3 + AP.pitch_kp * (tf+2)->b3);

	//done till here. continue from here.


}
