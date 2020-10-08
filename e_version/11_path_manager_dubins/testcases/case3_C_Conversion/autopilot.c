
#include <stdio.h>
#include "math_util.h"
#include "functions_vtol.h"
#include "parameters_vtol.h"

void autopilot(float *states_estimated, float* guidance_commands, struct ap_gains AP, struct actuators u_trim, float t, float* autopilot_commands)
{
	//int NN = 0;
	float pn       =  *(states_estimated+0); // inertial North position
	float pe       =  *(states_estimated+1); // inertial East position
	float h        =  *(states_estimated+2); // altitude
	float Va       =  *(states_estimated+3); // airspeed
	float alpha    =  *(states_estimated+4); // angle of attack
	float beta     =  *(states_estimated+5); // side slip angle
	float phi      =  *(states_estimated+6); // roll angle
	float theta    =  *(states_estimated+7); // pitch angle
	float chi      =  *(states_estimated+8); // course angle
	float p        =  *(states_estimated+9); // body frame roll rate
	float q        =  *(states_estimated+10);// body frame pitch rate
	float r        =  *(states_estimated+11);// body frame yaw rate
	float Vg       =  *(states_estimated+12);// ground speed
	float wn       =  *(states_estimated+13);// wind North
	float we       =  *(states_estimated+14);// wind East
	float psi      =  *(states_estimated+15);// heading
	float bx       =  *(states_estimated+16);// x-gyro bias
	float by       =  *(states_estimated+17);// y-gyro bias
	float bz       =  *(states_estimated+18);// z-gyro bias
	//NN = NN+18;
	float Va_c     = *(guidance_commands+0);  // commanded airspeed (m/s)
	float h_c      = *(guidance_commands+1);  // commanded altitude (m)
	float chi_c    = *(guidance_commands+2);  // commanded course (rad)
	float phi_c    = *(guidance_commands+3);
	float theta_c;

	//float t = uu[5+NN];

	static float intg_prev[7];
	static float diff_prev[7];
	static float err_prev[7];
	static float xi_prev;
	static float Ts_prev;
	static float Kr_prev;
	static float pWo_prev;

	float throttle_max = 2.0;
	float throttle_limit = 2.5;

	struct actuator_commands throtl, phiC, thetaC;
	float delta_e, delta_a, delta_r, delta_t;
	float * yaw_cmds;
	yaw_cmds = (float *)malloc(5*sizeof(float));

	int flag = 2;

	if(t==0)
	{
		flag = 1; // for initialization, flag is 1. For rest all cases it is 2;
	    
	    for(int i=0; i<7; i++)
	    {
	    	intg_prev[i] = 0.0;
		    diff_prev[i] = 0.0;
		    err_prev[i]  = 0.0;
	    }
	    	    
	    xi_prev = 0;
	    Ts_prev = vtol.Ts;
	    Kr_prev = AP.yaw_damper_kp;
	    pWo_prev = AP.p_wo;
	}
		
	if(flag==1)
	{
		phi_c   = 0.0;
    	delta_r = 0.0;
	}
	else
	{
		/*
		y_c = ;
		y 	= ;
		kp 	= ;
		ki 	= ;
		kd 	= ;
		limit = ;
		intg = ;
		diff = ;
		err = ;
		*/

		//course_with_roll
		phiC = pidloop(chi_c, chi, AP.course_kp, AP.course_ki, 0, 30*PI/180, vtol.Ts, AP.tau, intg_prev[0], diff_prev[0], err_prev[0]);
	    phi_c = phiC.actr_deflect_cmd;
	    intg_prev[0] = phiC.intg;
	    diff_prev[0] = phiC.diff;
	    err_prev[0]  = phiC.err;

	    //yaw_damper
	    yaw_cmds = yaw_damper(r, xi_prev, Ts_prev, Kr_prev, pWo_prev);
	    
	    delta_r = *(yaw_cmds+0); 
	    xi_prev = *(yaw_cmds+1); 
	    Ts_prev = *(yaw_cmds+2); 
	    Kr_prev = *(yaw_cmds+3); 
	    pWo_prev= *(yaw_cmds+4); 

	}

	//roll_with_aileron
	delta_a = pidloop_rate(phi_c, phi, p, AP.roll_kp, AP.roll_kd, 30*PI/180);

	float h_takeoff = 15;
	float h_hold = 20;
	float theta_takeoff = 15*PI/180;

if(h < h_takeoff)
{
	delta_t = throttle_max;
    theta_c = theta_takeoff;
}
//     h1 = h
else if(h >= h_takeoff && h < h_c - h_hold)
{
	delta_t = throttle_max;
    
	//airspeed_with_pitch
	thetaC = pidloop(Va_c, Va, AP.kp_V2, AP.ki_V2, 0, 30*PI/180, vtol.Ts, AP.tau, intg_prev[1], diff_prev[1], err_prev[1]);

	theta_c = thetaC.actr_deflect_cmd;
	intg_prev[1] = thetaC.intg;
	diff_prev[1] = thetaC.diff;
	err_prev[1]  = thetaC.err;
}
//%     h2= h
else if(h >= h_c-h_hold && h < h_c + h_hold)
{
	//airspeed_with_throttle
	throtl = pidloop(Va_c, Va, AP.kp_V, AP.ki_V, 0, throttle_limit, vtol.Ts, AP.tau, intg_prev[2], diff_prev[2], err_prev[2]);
	
	delta_t      = u_trim.delta_t + throtl.actr_deflect_cmd;
	intg_prev[2] = throtl.intg;
	diff_prev[2] = throtl.diff;
	err_prev[2]  = throtl.err;	

    if(delta_t < 0.0)
        delta_t = 0.0;
 
	//altitude_with_pitch
	thetaC = pidloop(h_c, h, AP.altitude_kp, AP.altitude_ki, 0, 30*PI/180, vtol.Ts, AP.tau, intg_prev[3], diff_prev[3], err_prev[3]);

	theta_c = thetaC.actr_deflect_cmd;
	intg_prev[3] = thetaC.intg;
	diff_prev[3] = thetaC.diff;
	err_prev[3]  = thetaC.err;
}  
// h3= h
else if(h >= h_c + h_hold)
{
	delta_t = 0.25;
    
    //airspeed_with_pitch
    thetaC = pidloop(Va_c, Va, AP.kp_V2, AP.ki_V2, 0, 30*PI/180, vtol.Ts, AP.tau, intg_prev[4], diff_prev[4], err_prev[4]);

	theta_c = thetaC.actr_deflect_cmd;
	intg_prev[4] = thetaC.intg;
	diff_prev[4] = thetaC.diff;
	err_prev[4]  = thetaC.err;
}
    
//pitch_with_elevator
	delta_e = pidloop_rate(theta_c, theta, q, AP.pitch_kp, AP.pitch_kd, 30*PI/180);


*(autopilot_commands+0) = delta_e;	
*(autopilot_commands+1) = delta_a;
*(autopilot_commands+2) = delta_r;   
*(autopilot_commands+3) = delta_t;
*(autopilot_commands+4) = 0.0;	
*(autopilot_commands+5) = 0.0;
*(autopilot_commands+6) = h_c;   
*(autopilot_commands+7) = Va_c;
*(autopilot_commands+8) = 0.0;	
*(autopilot_commands+9) = 0.0;
*(autopilot_commands+10) = phi_c;   
*(autopilot_commands+11) = theta_c;
*(autopilot_commands+12) = chi_c;	
*(autopilot_commands+13) = 0.0;
*(autopilot_commands+14) = 0.0;   
*(autopilot_commands+15) = 0.0;

free(yaw_cmds);

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// course_with_roll
//   - regulate heading using the roll command
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
struct actuator_commands pidloop(float y_c, float y, float kp, float ki, float kd, float limit, float Ts, float tau, float intg, float diff, float err)
{
	float integrator = intg;
	float differentiator = diff;
	float error_d1 = err;

	float error = y_c - y; // compute the current error
	 
	integrator = integrator + (Ts/2)*(error + error_d1);

	// update integrator
	differentiator = ((2*tau-Ts)/(2*tau+Ts))*differentiator + (2/(2*tau+Ts))*(error - error_d1);

	// update differentiator
	error_d1 = error; // update the error for next time through the loop
	 
	float u = sat(kp * error +  ki * integrator + kd * differentiator, limit); // ensure abs(u)<=limit
	 
	// implement integrator anti-windup
	if(ki != 0.0)
	{
		float u_unsat = kp*error + ki*integrator + kd*differentiator;
	    integrator = integrator + (1/ki) * (u - u_unsat);
	}
	     
	struct actuator_commands act_cmds = {u, integrator, differentiator, error_d1};

	return act_cmds;
}

float pidloop_rate(float y_c, float y, float ydot, float kp, float kd, float limit)
{
	float error = y_c - y; 
	float u = sat(kp * error - kd * ydot, limit);
	return u;
}

float * yaw_damper(float r, float xi, float Ts, float Kr, float p_wo)
{
	float * yaw_cmds;
	yaw_cmds = (float *)malloc(5*sizeof(float));

	xi = xi + Ts*(-p_wo*xi + Kr*r);
	float delta_r = sat(-p_wo*xi + Kr*r, 20*PI/180);

	float xi_new = xi;
	float Ts_new = Ts;
	float Kr_new = Kr;
	float pWo_new = p_wo;

	*(yaw_cmds+0) = delta_r;
	*(yaw_cmds+1) = xi_new;
	*(yaw_cmds+2) = Ts_new;
	*(yaw_cmds+3) = Kr_new;
	*(yaw_cmds+4) = pWo_new;

	return yaw_cmds;
}

float sat(float in, float limit)
{
	float out;
	if(in < -limit)
        out = -limit;
    else if(in > limit)
        out = limit;
    else{
        out = in;
    }
    
    return out;
}

