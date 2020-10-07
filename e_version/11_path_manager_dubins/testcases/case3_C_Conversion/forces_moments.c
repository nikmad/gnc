
#include "math_util.h"
#include "functions_vtol.h"
#include "parameters_vtol.h"

struct force_n_moments forces_moments(struct states states_in, struct actuators delta, struct wnd wind)
{
	struct force_n_moments fm_out;

	float *R_v_v1;
	R_v_v1 = (float *) malloc(9*sizeof(float));

	*R_v_v1 	= cosf(states_in.psi);
	*(R_v_v1+1) = sinf(states_in.psi);
	*(R_v_v1+2) = 0;
	
	*(R_v_v1+3) = -sinf(states_in.psi);
	*(R_v_v1+4) = cosf(states_in.psi);
	*(R_v_v1+5) = 0;
	
	*(R_v_v1+6) = 0;
	*(R_v_v1+7) = 0;
	*(R_v_v1+8) = 1;

	float *R_v1_v2;
	R_v1_v2 = (float *) malloc(9*sizeof(float));

	*R_v1_v2   	 = cosf(states_in.theta);
	*(R_v1_v2+1) = 0;
	*(R_v1_v2+2) = -sinf(states_in.theta);
	
	*(R_v1_v2+3) = 0;
	*(R_v1_v2+4) = 1;
	*(R_v1_v2+5) = 0;
	
	*(R_v1_v2+6) = sinf(states_in.theta);
	*(R_v1_v2+7) = 0;
	*(R_v1_v2+8) = cosf(states_in.theta);

	float *R_v2_b;
	R_v2_b = (float *) malloc(9*sizeof(float));

	*R_v2_b   = 1;
	*(R_v2_b+1) = 0;
	*(R_v2_b+2) = 0;
	
	*(R_v2_b+3) = 0;
	*(R_v2_b+4) = cosf(states_in.phi);
	*(R_v2_b+5) = sinf(states_in.phi);
	
	*(R_v2_b+6) = 0;
	*(R_v2_b+7) = -sinf(states_in.phi);
	*(R_v2_b+8) = cosf(states_in.phi);

	float *temp3X3_1;
	temp3X3_1 = (float *) malloc(9*sizeof(float));

	float *R_v_b;
	R_v_b = (float *) malloc(9*sizeof(float));

	//Creating the rotation matrix
	MatrixMultiply(R_v1_v2,3,3,R_v_v1,3,3,temp3X3_1);
	MatrixMultiply(R_v2_b,3,3,temp3X3_1,3,3,R_v_b);

	//FILE *fptr1;
	//fptr1 = fopen("matmulC.txt", "a");
	//fprintf(fptr1, "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f ", wind.w_ns, wind.w_es, wind.w_ds, wind.u_wg, wind.v_wg, wind.w_wg);
	//fprintf(fptr1, "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n", R_v_b[0][0], R_v_b[0][1], R_v_b[0][2], R_v_b[1][0], R_v_b[1][1], R_v_b[1][2], R_v_b[2][0], R_v_b[2][1], R_v_b[2][2]);
	//fclose(fptr1);

	float *temp3x1_1, *temp3x1_2;
	temp3x1_1 = (float *) malloc(3*sizeof(float));
	temp3x1_2 = (float *) malloc(3*sizeof(float));

	*temp3x1_1 		= wind.w_ns;
	*(temp3x1_1+1) 	= wind.w_es;
	*(temp3x1_1+2) 	= wind.w_ds;

	//Converting steady wind from NED to body frame
	MatrixMultiply(R_v_b,3,3,temp3x1_1,3,1,temp3x1_2);

	//Total wind vector in body-frame: adding the steady components and wind gust components in body frame
	float V_w_MAT[3][1];
	V_w_MAT[0][0] = *temp3x1_2 		+ wind.u_wg;
	V_w_MAT[1][0] = *(temp3x1_2+1) 	+ wind.v_wg;
	V_w_MAT[2][0] = *(temp3x1_2+2) 	+ wind.w_wg;

	//Body-frame components of the airspeed vector
	float u_r, v_r, w_r;
	u_r = states_in.u - V_w_MAT[0][0];
	v_r = states_in.v - V_w_MAT[1][0];
	w_r = states_in.w - V_w_MAT[2][0];

	//compute air data
	fm_out.Va = sqrtf(u_r*u_r+v_r*v_r+w_r*w_r);
	fm_out.alpha = (float)atan2f(w_r,u_r);
	fm_out.beta = asinf(v_r/fm_out.Va);

	//Total wind vector in NED frame
	float *V_w, *V_v;
	V_w = (float *) malloc(3*sizeof(float));
	V_v = (float *) malloc(3*sizeof(float));

	float R_v_b_MAT[3][3];
	
	R_v_b_MAT[0][0] = *(R_v_b);  
	R_v_b_MAT[0][1] = *(R_v_b+1);
	R_v_b_MAT[0][2] = *(R_v_b+2);
	                  
	R_v_b_MAT[1][0] = *(R_v_b+3);
	R_v_b_MAT[1][1] = *(R_v_b+4);
	R_v_b_MAT[1][2] = *(R_v_b+5);
	                  
	R_v_b_MAT[2][0] = *(R_v_b+6);
	R_v_b_MAT[2][1] = *(R_v_b+7);
	R_v_b_MAT[2][2] = *(R_v_b+8);

	transposed3x3(R_v_b_MAT); //this function expects matrix hence had to convert from pointer R_v_b to matrix R_v_b_MAT

	float *R_b_v;
    R_b_v = (float *) malloc(9*sizeof(float));

    *(R_b_v)   = R_v_b_MAT[0][0];  
    *(R_b_v+1) = R_v_b_MAT[0][1];  
    *(R_b_v+2) = R_v_b_MAT[0][2];  
                                  
    *(R_b_v+3) = R_v_b_MAT[1][0];  
    *(R_b_v+4) = R_v_b_MAT[1][1];  
    *(R_b_v+5) = R_v_b_MAT[1][2];  
                                  
    *(R_b_v+6) = R_v_b_MAT[2][0];  
    *(R_b_v+7) = R_v_b_MAT[2][1];  
    *(R_b_v+8) = R_v_b_MAT[2][2];  

    *V_w 	 = V_w_MAT[0][0];
    *(V_w+1) = V_w_MAT[1][0];
    *(V_w+2) = V_w_MAT[2][0];	
 
	MatrixMultiply(R_b_v,3,3,V_w,3,1,V_v);

	fm_out.w_n = *V_v;    
	fm_out.w_e = *(V_v+1);
	fm_out.w_d = *(V_v+2);

	free(R_v_v1);
	free(R_v1_v2);
	free(R_v2_b);
	free(temp3X3_1);
	free(R_v_b);
	free(temp3x1_1);
	free(temp3x1_2);
	free(V_w);
	free(V_v);
	free(R_b_v);

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

    float CDalpha = vtol.CDp  + (powf(vtol.CL0 + vtol.CLalpha*fabsf(fm_out.alpha),2))/(PI*vtol.e*vtol.AR);
   	float sigma_alpha = (1 + expf(-vtol.M*(fm_out.alpha-vtol.alpha0)) + expf(vtol.M*(fm_out.alpha+vtol.alpha0)))/((1+expf(-vtol.M*(fm_out.alpha-vtol.alpha0)))*(1+expf(vtol.M*(fm_out.alpha+vtol.alpha0))));
   	float CLalpha = (1-sigma_alpha)*(vtol.CL0 + vtol.CLalpha*fm_out.alpha) + sigma_alpha*(2*sign(fm_out.alpha)*powf(sinf(fm_out.alpha),2)*cosf(fm_out.alpha));
    
    float CXalpha = -CDalpha*cosf(fm_out.alpha) + CLalpha*sinf(fm_out.alpha);
    float CXq_alpha = -vtol.CDq*cosf(fm_out.alpha) + vtol.CLq*sinf(fm_out.alpha);
    float CXdelta_e_alpha = -vtol.CDdelta_e*cosf(fm_out.alpha) + vtol.CLdelta_e*sinf(fm_out.alpha);
    float CZalpha = -CDalpha*sinf(fm_out.alpha) - CLalpha*cosf(fm_out.alpha);
    float CZq_alpha = -vtol.CDq*sinf(fm_out.alpha) - vtol.CLq*cosf(fm_out.alpha);
    float CZdelta_e_alpha = -vtol.CDdelta_e*sinf(fm_out.alpha) - vtol.CLdelta_e*cosf(fm_out.alpha);

    float fa_x = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*(CXalpha + CXq_alpha*(vtol.c/(2*fm_out.Va))*states_in.q + CXdelta_e_alpha*fabsf(delta.delta_e)); 
    float fa_y = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*(vtol.CY0 + vtol.CYbeta*fm_out.beta + vtol.CYp*(vtol.b/(2*fm_out.Va))*states_in.p + vtol.CYr*(vtol.b/(2*fm_out.Va))*states_in.r + vtol.CYdelta_a*delta.delta_a + vtol.CYdelta_r*delta.delta_r);
    float fa_z = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*(CZalpha + CZq_alpha*(vtol.c/(2*fm_out.Va))*states_in.q + CZdelta_e_alpha*delta.delta_e);

    //====================================================
    // Engines/Motors(propulsion system) forces
    //====================================================
    float C_prop = 1.0;
    float k_motor = 80;

    float Vin = vtol.Vmax * delta.delta_t;

    //parameters of quadratic equation solution
    float a_omega = vtol.rho*powf(vtol.D_prop,5)*vtol.CQ0/powf(2*PI,2);
    float b_omega = vtol.rho*powf(vtol.D_prop,4)*vtol.CQ1*fm_out.Va/(2*PI) + powf(vtol.KQ,2)/vtol.Rmotor;
    float c_omega = vtol.rho*powf(vtol.D_prop,3)*vtol.CQ2*powf(fm_out.Va,2) - vtol.KQ * Vin/vtol.Rmotor + vtol.KQ * vtol.i0;
    
    float Omega_p = (-b_omega + sqrtf(powf(b_omega,2)-4*a_omega*c_omega))/(2*a_omega);
    float Tp = vtol.rho*powf(vtol.D_prop,4)*vtol.CT0*powf(Omega_p,2)/(4*PI*PI)+ vtol.rho*powf(vtol.D_prop,3)*vtol.CT1*fm_out.Va*Omega_p/(2*PI) + vtol.rho*powf(vtol.D_prop,2)*vtol.CT2*powf(fm_out.Va,2);
    float Qp = vtol.rho*powf(vtol.D_prop,5)*vtol.CQ0*powf(Omega_p,2)/(4*PI*PI)+ vtol.rho*powf(vtol.D_prop,4)*vtol.CQ1*fm_out.Va*Omega_p/(2*PI) + vtol.rho*powf(vtol.D_prop,3)*vtol.CQ2*powf(fm_out.Va,2);

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
//struct force_n_moments forces_moments(struct states states_in, struct actuators delta, struct wnd wind)
//{
//	struct force_n_moments fm_out;
//
//	float R_v_v1[3][3];
//	R_v_v1[0][0] = cosf(states_in.psi);
//	R_v_v1[0][1] = sinf(states_in.psi);
//	R_v_v1[0][2] = 0;
//	
//	R_v_v1[1][0] = -sinf(states_in.psi);
//	R_v_v1[1][1] = cosf(states_in.psi);
//	R_v_v1[1][2] = 0;
//	
//	R_v_v1[2][0] = 0;
//	R_v_v1[2][1] = 0;
//	R_v_v1[2][2] = 1;
//
//	float R_v1_v2[3][3];
//	R_v1_v2[0][0] = cosf(states_in.theta);
//	R_v1_v2[0][1] = 0;
//	R_v1_v2[0][2] = -sinf(states_in.theta);
//	
//	R_v1_v2[1][0] = 0;
//	R_v1_v2[1][1] = 1;
//	R_v1_v2[1][2] = 0;
//	
//	R_v1_v2[2][0] = sinf(states_in.theta);
//	R_v1_v2[2][1] = 0;
//	R_v1_v2[2][2] = cosf(states_in.theta);
//
//	float R_v2_b[3][3];
//	R_v2_b[0][0] = 1;
//	R_v2_b[0][1] = 0;
//	R_v2_b[0][2] = 0;
//	
//	R_v2_b[1][0] = 0;
//	R_v2_b[1][1] = cosf(states_in.phi);
//	R_v2_b[1][2] = sinf(states_in.phi);
//	
//	R_v2_b[2][0] = 0;
//	R_v2_b[2][1] = -sinf(states_in.phi);
//	R_v2_b[2][2] = cosf(states_in.phi);
//
//	float temp3X3_1[3][3];
//	float R_v_b[3][3];
//
//	//Creating the rotation matrix
//	MatrixMultiply(R_v1_v2,3,3,R_v_v1,3,3,temp3X3_1);
//	MatrixMultiply(R_v2_b,3,3,temp3X3_1,3,3,R_v_b);
//
//	//FILE *fptr1;
//	//fptr1 = fopen("matmulC.txt", "a");
//	//fprintf(fptr1, "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f ", wind.w_ns, wind.w_es, wind.w_ds, wind.u_wg, wind.v_wg, wind.w_wg);
//	//fprintf(fptr1, "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n", R_v_b[0][0], R_v_b[0][1], R_v_b[0][2], R_v_b[1][0], R_v_b[1][1], R_v_b[1][2], R_v_b[2][0], R_v_b[2][1], R_v_b[2][2]);
//	//fclose(fptr1);
//
//	float temp3x1_1[3][1], temp3x1_2[3][1];
//	temp3x1_1[0][0] = wind.w_ns;
//	temp3x1_1[1][0] = wind.w_es;
//	temp3x1_1[2][0] = wind.w_ds;
//
//	//Converting steady wind from NED to body frame
//	MatrixMultiply(R_v_b,3,3,temp3x1_1,3,1,temp3x1_2);
//
//	//Total wind vector in body-frame: adding the steady components and wind gust components in body frame
//	float V_w[3][1];
//	V_w[0][0] = temp3x1_2[0][0] + wind.u_wg;
//	V_w[1][0] = temp3x1_2[1][0] + wind.v_wg;
//	V_w[2][0] = temp3x1_2[2][0] + wind.w_wg;
//
//	//Body-frame components of the airspeed vector
//	float u_r, v_r, w_r;
//	u_r = states_in.u - V_w[0][0];
//	v_r = states_in.v - V_w[1][0];
//	w_r = states_in.w - V_w[2][0];
//
//	//compute air data
//	fm_out.Va = sqrt(u_r*u_r+v_r*v_r+w_r*w_r);
//	fm_out.alpha = (float)atan2(w_r,u_r);
//	fm_out.beta = asin(v_r/fm_out.Va);
//
//	//Total wind vector in NED frame
//	float V_v[3][1];
//	transposed3x3(R_v_b);
//	float R_b_v[3][3];
//
//	//R_b_v[][] = R_v_b[][];
//
//	int i, j;
//	for(i=0; i<3; i++)
//	for(j=0; j<3; j++)
//	{
//		R_b_v[i][j] = R_v_b[i][j];
//	}
//
//	MatrixMultiply(R_b_v,3,3,V_w,3,1,V_v);
//
//	fm_out.w_n = V_v[0][0];
//	fm_out.w_e = V_v[1][0];
//	fm_out.w_d = V_v[2][0];
//
//	// compute external forces and torques on aircraft
//
//    //====================================================
//    // Gravity force
//    //====================================================
//	float fg_x, fg_y, fg_z;
//
//	fg_x = -vtol.m * vtol.g * sinf(states_in.theta);
//	fg_y = vtol.m * vtol.g * cosf(states_in.theta) * sinf(states_in.phi);
//	fg_z = vtol.m * vtol.g * cosf(states_in.theta) * cosf(states_in.phi);
//
//	//====================================================
//    // Aerodynamic forces
//    //====================================================
//
//    float CDalpha = vtol.CDp  + (powf(vtol.CL0 + vtol.CLalpha*abs(fm_out.alpha),2))/(PI*vtol.e*vtol.AR);
//   	float sigma_alpha = (1 + exp(-vtol.M*(fm_out.alpha-vtol.alpha0)) + exp(vtol.M*(fm_out.alpha+vtol.alpha0)))/((1+exp(-vtol.M*(fm_out.alpha-vtol.alpha0)))*(1+exp(vtol.M*(fm_out.alpha+vtol.alpha0))));
//   	float CLalpha = (1-sigma_alpha)*(vtol.CL0 + vtol.CLalpha*fm_out.alpha) + sigma_alpha*(2*sign(fm_out.alpha)*powf(sinf(fm_out.alpha),2)*cosf(fm_out.alpha));
//    
//    float CXalpha = -CDalpha*cosf(fm_out.alpha) + CLalpha*sinf(fm_out.alpha);
//    float CXq_alpha = -vtol.CDq*cosf(fm_out.alpha) + vtol.CLq*sinf(fm_out.alpha);
//    float CXdelta_e_alpha = -vtol.CDdelta_e*cosf(fm_out.alpha) + vtol.CLdelta_e*sinf(fm_out.alpha);
//    float CZalpha = -CDalpha*sinf(fm_out.alpha) - CLalpha*cosf(fm_out.alpha);
//    float CZq_alpha = -vtol.CDq*sinf(fm_out.alpha) - vtol.CLq*cosf(fm_out.alpha);
//    float CZdelta_e_alpha = -vtol.CDdelta_e*sinf(fm_out.alpha) - vtol.CLdelta_e*cosf(fm_out.alpha);
//
//    float fa_x = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*(CXalpha + CXq_alpha*(vtol.c/(2*fm_out.Va))*states_in.q + CXdelta_e_alpha*abs(delta.delta_e)); 
//    float fa_y = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*(vtol.CY0 + vtol.CYbeta*fm_out.beta + vtol.CYp*(vtol.b/(2*fm_out.Va))*states_in.p + vtol.CYr*(vtol.b/(2*fm_out.Va))*states_in.r + vtol.CYdelta_a*delta.delta_a + vtol.CYdelta_r*delta.delta_r);
//    float fa_z = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*(CZalpha + CZq_alpha*(vtol.c/(2*fm_out.Va))*states_in.q + CZdelta_e_alpha*delta.delta_e);
//
//    //====================================================
//    // Engines/Motors(propulsion system) forces
//    //====================================================
//    float C_prop = 1.0;
//    float k_motor = 80;
//
//    float Vin = vtol.Vmax * delta.delta_t;
//
//    //parameters of quadratic equation solution
//    float a_omega = vtol.rho*powf(vtol.D_prop,5)*vtol.CQ0/powf(2*PI,2);
//    float b_omega = vtol.rho*powf(vtol.D_prop,4)*vtol.CQ1*fm_out.Va/(2*PI) + powf(vtol.KQ,2)/vtol.Rmotor;
//    float c_omega = vtol.rho*powf(vtol.D_prop,3)*vtol.CQ2*powf(fm_out.Va,2) - vtol.KQ * Vin/vtol.Rmotor + vtol.KQ * vtol.i0;
//    
//    float Omega_p = (-b_omega + sqrt(powf(b_omega,2)-4*a_omega*c_omega))/(2*a_omega);
//    float Tp = vtol.rho*powf(vtol.D_prop,4)*vtol.CT0*powf(Omega_p,2)/(4*PI*PI)+ vtol.rho*powf(vtol.D_prop,3)*vtol.CT1*fm_out.Va*Omega_p/(2*PI) + vtol.rho*powf(vtol.D_prop,2)*vtol.CT2*powf(fm_out.Va,2);
//    float Qp = vtol.rho*powf(vtol.D_prop,5)*vtol.CQ0*powf(Omega_p,2)/(4*PI*PI)+ vtol.rho*powf(vtol.D_prop,4)*vtol.CQ1*fm_out.Va*Omega_p/(2*PI) + vtol.rho*powf(vtol.D_prop,3)*vtol.CQ2*powf(fm_out.Va,2);
//
//    float fp_x = Tp; 
//    float fp_y = 0.0;
//    float fp_z = 0.0;
//
//    //====================================================
//    // Total Force in Body frame
//    //====================================================
//    fm_out.fx =  fg_x+fa_x+fp_x; // fx
//    fm_out.fy =  fg_y+fa_y+fp_y; // fy
//    fm_out.fz =  fg_z+fa_z+fp_z; // fz
//
//    //====================================================
//    // Aerodynamic Moments
//    //====================================================
//    float ell_a = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*vtol.b*(vtol.Cl0 + vtol.Clbeta*fm_out.beta + vtol.Clp*(vtol.b/(2*fm_out.Va))*states_in.p + vtol.Clr*(vtol.b/(2*fm_out.Va))*states_in.r + vtol.Cldelta_a*delta.delta_a + vtol.Cldelta_r*delta.delta_r);
//    float m_a = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*vtol.c*(vtol.Cm0 + vtol.Cmalpha*fm_out.alpha + vtol.Cmq*(vtol.c/(2*fm_out.Va))*states_in.q + vtol.Cmdelta_e*delta.delta_e);
//    float n_a = 0.5*vtol.rho*powf(fm_out.Va,2)*vtol.S_wing*vtol.b*(vtol.Cn0 + vtol.Cnbeta*fm_out.beta + vtol.Cnp*(vtol.b/(2*fm_out.Va))*states_in.p + vtol.Cnr*(vtol.b/(2*fm_out.Va))*states_in.r + vtol.Cndelta_a*delta.delta_a + vtol.Cndelta_r*delta.delta_r);
//
//    //====================================================
//    // Engines/motors(propulsion system) Moments
//    //====================================================
//    float k_T_p = 0;
//    float k_omega = 0;
//
//    float ell_p = Qp;
//    float m_p = 0;
//    float n_p = 0;
//
//    //====================================================
//    // Total Moments in Body frame
//    //====================================================
//
//     fm_out.l = ell_a+ell_p;
//     fm_out.m = m_a+m_p;   
//     fm_out.n = n_a+n_p;
//
//     return fm_out;
//}


