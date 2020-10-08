/*
Author: Nikhil Madduri (nikhil.madduri@gmail.com)
Created: 31/Jul/2020
*/
//#define EXTERN
//#define VTOL

#include <stdio.h>
#include <math.h>
#include "parameters_vtol.h"
#include "functions_vtol.h"
#include "math_util.h"
#define WAYPOINT_SIZE 100

//_______________________________________________________
int main()
{
	//float rk4_stepsize = 0.01;

	struct states states_in = {0,0,0,0,0,0,0,0,0,0,0,0};
	struct states states_out, states_prevMemory; 
	
	float* states_estimated;
	states_estimated = (float*)malloc(19*sizeof(float));

	struct force_n_moments fm_in = {0,0,0, 0,0,0, 0,0,0, 0,0,0};
	struct actuators delta = {0*PI/180, 0*PI/180, 0*PI/180, 0};
	struct wnd _wind = {0.0000000001,0.0000000001,0.0000000001,0.0000000001,0.0000000001,0.0000000001};
	
	struct trans_funcs* tf;
	tf = (struct trans_funcs*)malloc(8*sizeof(struct trans_funcs));

	float* guidance_commands;
	guidance_commands = (float*)malloc(4*sizeof(float));
	
	float* autopilot_commands;
	autopilot_commands = (float*)malloc(16*sizeof(float));

	int i;
	float t = 0.0, t_tot = 300.0;

	float chi = 0.0;

	struct states x_trim = {0, 0, 0, 34.99, 0, 0.082, -0.00022, -0.26, 0, 0, 0, 0};
	struct trim_out y_trim = {35, 0.0023, 0};
	struct actuators u_trim = {0.0072, 0.0012, -0.00019, 0.85};

	// Initial states set to trim values
	states_in = x_trim;
	states_in.pn = 0;
	states_in.pe = 0;
	states_in.pd = -140;

	tf = transfer_functions(x_trim, y_trim, u_trim);

	FILE *fptr_tf;
	fptr_tf = fopen("transfer_functions.txt", "w+");
	
	fprintf(fptr_tf, "    		b0\t\t b1\t\t b2\t\t b3\t\t a0\t\t a1\t\t a2\t\t a3\n\n");
	fprintf(fptr_tf, "tf_phiDelA  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",    tf->b0,     tf->b1,     tf->b2,     tf->b3,     tf->a0,     tf->a1,     tf->a2,     tf->a3);
	fprintf(fptr_tf, "tf_chiPhi  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+1)->b0, (tf+1)->b1, (tf+1)->b2, (tf+1)->b3, (tf+1)->a0, (tf+1)->a1, (tf+1)->a2, (tf+1)->a3);
	fprintf(fptr_tf, "tf_thetaDelE  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+2)->b0, (tf+2)->b1, (tf+2)->b2, (tf+2)->b3, (tf+2)->a0, (tf+2)->a1, (tf+2)->a2, (tf+2)->a3);
	fprintf(fptr_tf, "tf_hTheta  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+3)->b0, (tf+3)->b1, (tf+3)->b2, (tf+3)->b3, (tf+3)->a0, (tf+3)->a1, (tf+3)->a2, (tf+3)->a3);
	fprintf(fptr_tf, "tf_hVa  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+4)->b0, (tf+4)->b1, (tf+4)->b2, (tf+4)->b3, (tf+4)->a0, (tf+4)->a1, (tf+4)->a2, (tf+4)->a3);
	fprintf(fptr_tf, "tf_VaDelT  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+5)->b0, (tf+5)->b1, (tf+5)->b2, (tf+5)->b3, (tf+5)->a0, (tf+5)->a1, (tf+5)->a2, (tf+5)->a3);
	fprintf(fptr_tf, "tf_VaTheta 	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+6)->b0, (tf+6)->b1, (tf+6)->b2, (tf+6)->b3, (tf+6)->a0, (tf+6)->a1, (tf+6)->a2, (tf+6)->a3);
	fprintf(fptr_tf, "tf_vDelR  	%+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\t %+f\n",(tf+7)->b0, (tf+7)->b1, (tf+7)->b2, (tf+7)->b3, (tf+7)->a0, (tf+7)->a1, (tf+7)->a2, (tf+7)->a3);

	fclose(fptr_tf);

	struct ap_gains AP;
	//AP = (struct ap_gains*)malloc(sizeof(struct ap_gains));
	AP = gains_autopilot(x_trim, y_trim, u_trim, tf);

	FILE *fptr_ap;
	fptr_ap = fopen("ap_gains.txt", "w+");
    
	fprintf(fptr_ap, " AP.tau               %+f\n", AP.tau              );
	fprintf(fptr_ap, " AP.delta_a_max       %+f\n", AP.delta_a_max      );
	fprintf(fptr_ap, " AP.e_phi_max         %+f\n", AP.e_phi_max        );
	fprintf(fptr_ap, " AP.zeta_phi          %+f\n", AP.zeta_phi         );
	fprintf(fptr_ap, " AP.roll_kp           %+f\n", AP.roll_kp          );
	fprintf(fptr_ap, " AP.omega_n_phi       %+f\n", AP.omega_n_phi      );
	fprintf(fptr_ap, " AP.roll_kd           %+f\n", AP.roll_kd          );
	fprintf(fptr_ap, " AP.delta_r_max       %+f\n", AP.delta_r_max      );
	fprintf(fptr_ap, " AP.e_beta_max        %+f\n", AP.e_beta_max       );
	fprintf(fptr_ap, " AP.zeta_sideslip     %+f\n", AP.zeta_sideslip    );
	fprintf(fptr_ap, " AP.sideslip_kp       %+f\n", AP.sideslip_kp      );
	fprintf(fptr_ap, " AP.sideslip_ki       %+f\n", AP.sideslip_ki      );
	fprintf(fptr_ap, " AP.d_beta            %+f\n", AP.d_beta           );
	fprintf(fptr_ap, " AP.omega_n_dr        %+f\n", AP.omega_n_dr       );
	fprintf(fptr_ap, " AP.p_wo              %+f\n", AP.p_wo             );
	fprintf(fptr_ap, " AP.yaw_damper_tau_r  %+f\n", AP.yaw_damper_tau_r );
	fprintf(fptr_ap, " AP.yaw_damper_kp     %+f\n", AP.yaw_damper_kp );
	fprintf(fptr_ap, " AP.zeta_course       %+f\n", AP.zeta_course   );
	fprintf(fptr_ap, " AP.omega_n_chi       %+f\n", AP.omega_n_chi   );
	fprintf(fptr_ap, " AP.course_kp         %+f\n", AP.course_kp     );
	fprintf(fptr_ap, " AP.course_ki         %+f\n", AP.course_ki     );
	fprintf(fptr_ap, " AP.d_chi             %+f\n", AP.d_chi         );
	fprintf(fptr_ap, " AP.delta_e_max       %+f\n", AP.delta_e_max   );
	fprintf(fptr_ap, " AP.e_theta_max       %+f\n", AP.e_theta_max   );
	fprintf(fptr_ap, " AP.zeta_theta        %+f\n", AP.zeta_theta    );
	fprintf(fptr_ap, " AP.pitch_kp          %+f\n", AP.pitch_kp      );
	fprintf(fptr_ap, " AP.omega_n_theta     %+f\n", AP.omega_n_theta );
	fprintf(fptr_ap, " AP.pitch_kd   	      %+f\n", AP.pitch_kd  );
	fprintf(fptr_ap, " AP.K_theta_DC 	      %+f\n", AP.K_theta_DC);
	fprintf(fptr_ap, " AP.W_h        	      %+f\n", AP.W_h       );
	fprintf(fptr_ap, " AP.zeta_h     	      %+f\n", AP.zeta_h    );
	fprintf(fptr_ap, " AP.omega_n_h  	      %+f\n", AP.omega_n_h );
	fprintf(fptr_ap, " AP.altitude_ki       %+f\n", AP.altitude_ki);
	fprintf(fptr_ap, " AP.altitude_kp       %+f\n", AP.altitude_kp);
	fprintf(fptr_ap, " AP.d_h        	      %+f\n", AP.d_h       );
	fprintf(fptr_ap, " AP.W_V2       	      %+f\n", AP.W_V2      );
	fprintf(fptr_ap, " AP.zeta_V2    	      %+f\n", AP.zeta_V2   );
	fprintf(fptr_ap, " AP.omega_n_V2 	      %+f\n", AP.omega_n_V2);
	fprintf(fptr_ap, " AP.kp_V2      	      %+f\n", AP.kp_V2     );
	fprintf(fptr_ap, " AP.ki_V2      	      %+f\n", AP.ki_V2     );
	fprintf(fptr_ap, " AP.zeta_V     	      %+f\n", AP.zeta_V    );
	fprintf(fptr_ap, " AP.omega_n_V  	      %+f\n", AP.omega_n_V );
	fprintf(fptr_ap, " AP.ki_V       	      %+f\n", AP.ki_V );
	fprintf(fptr_ap, " AP.kp_V       	      %+f\n", AP.kp_V );

	fclose(fptr_ap);

	FILE *fptr, *fptr_truestates, *fptr_autopilot;
	fptr_truestates = fopen("true_states.txt", "w+");
	fptr = fopen("nikstates.txt", "w+");
	fptr_autopilot = fopen("autopilot_commands.txt", "w+");

	//fprintf(fptr, "%3.3f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", t, states_in.pn, states_in.pe, states_in.pd, states_in.u, states_in.v, states_in.w, states_in.phi, states_in.theta, states_in.psi, states_in.p, states_in.q, states_in.r, fm_in.Va, fm_in.alpha,fm_in.beta,chi,delta.delta_e,delta.delta_a,delta.delta_r,delta.delta_t);

	for(i=0; i<(int)(t_tot/SIM.rk4_stepsize)+1; i++)
	{
		//printf("_____________________________________________\n");
		t = i*SIM.rk4_stepsize;
		fm_in = forces_moments(states_in, delta, _wind);
		states_out = vtol_dynamics(states_in, fm_in);
		chi = (float)atan2f((fm_in.Va*sinf(states_out.psi)+fm_in.w_e),(fm_in.Va*cosf(states_out.psi)+fm_in.w_n));
		//               1     2    3    4    5    6    7    8    9     10  11   12   13   14    15   16   17  18   19   20   21
		//               t     pn   pe   pd   u    v    w   phi  theta psi  p    q    r    Va  alpha beta chi delE delA delR delT
		fprintf(fptr, "%3.3f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", t, states_out.pn, states_out.pe, states_out.pd, states_out.u, states_out.v, states_out.w, states_out.phi, states_out.theta, states_out.psi, states_out.p, states_out.q, states_out.r, fm_in.Va, fm_in.alpha, fm_in.beta,chi,delta.delta_e,delta.delta_a,delta.delta_r,delta.delta_t);
		states_in = states_out;

		true_states(states_out, fm_in, states_estimated);
		//                        1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19
		fprintf(fptr_truestates, "%f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n", *states_estimated, *(states_estimated+1), *(states_estimated+2), *(states_estimated+3), *(states_estimated+4), *(states_estimated+5), *(states_estimated+6), *(states_estimated+7), *(states_estimated+8), *(states_estimated+9), *(states_estimated+10), *(states_estimated+11), *(states_estimated+12), *(states_estimated+13), *(states_estimated+14), *(states_estimated+15), *(states_estimated+16), *(states_estimated+17), *(states_estimated+18));

		guidance(guidance_commands);
		//printf(" Va_c = %f\n h_c = %f\n chi_c = %f\n phi_ff = %f\n", *guidance_commands, *(guidance_commands+1), *(guidance_commands+2), *(guidance_commands+3));

		autopilot(states_estimated, guidance_commands, AP, u_trim, t, autopilot_commands);
		//                        1   2    3    4    5    6    7    8    9    10   11   12   13   14   15   16
		fprintf(fptr_autopilot, "%f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f   %f\n",*(autopilot_commands), *(autopilot_commands+1), *(autopilot_commands+2), *(autopilot_commands+3),*(autopilot_commands+4), *(autopilot_commands+5), *(autopilot_commands+6), *(autopilot_commands+7),*(autopilot_commands+8), *(autopilot_commands+9), *(autopilot_commands+10), *(autopilot_commands+11),*(autopilot_commands+12), *(autopilot_commands+13), *(autopilot_commands+14), *(autopilot_commands+15));
   	}

	fclose(fptr);
	fclose(fptr_truestates);
	fclose(fptr_autopilot);

	free(tf);
	free(states_estimated);
	free(guidance_commands);
	free(autopilot_commands);
	
	return 0;
}

