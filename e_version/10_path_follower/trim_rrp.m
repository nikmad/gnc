function [alpha_trim,beta_trim,delta_e_trim,delta_a_trim,delta_r_trim,Tx_trim,stall_speed]= trim_rrp(V_trim)
% b1=propulsion_data(1); % b and d of vtol propellers
% d1=propulsion_data(2);
% b1_inv=1.0/b1;
%
rho=1.225; % density
%
V=V_trim;
% mass=structure_data(1,1);s=structure_data(1,4);
mass = 11.0;
s = 0.55;

% vtol.gravity = 9.81;
% 	vtol.mass = 11.0;
% % 	vtol.mass = 2477.0;
% 	vtol.Jx   = 0.8244;
% 	vtol.Jy   = 1.135;
% 	vtol.Jz   = 1.759;
% 	vtol.Jxz  = 0.1204;
% 	vtol.S_wing        = 0.55;
% 	vtol.b             = 2.90;
% 	vtol.c             = 0.19;
% % 	vtol.S_wing        = 21.55;
% % 	vtol.b             = 21.90;
% % 	vtol.c             = 2.19;
% %     vtol.Jx   = 2000;
% % 	vtol.Jy   = 2000;
% % 	vtol.Jz   = 2500;
% % 	vtol.Jxz  = 0;
%     
% 	vtol.S_prop        = 0.2027;
% 	vtol.rho           = 1.2682;
% 	vtol.e             = 0.9;
% 	vtol.AR            = vtol.b^2/vtol.S_wing;
%     
%  	% Gamma parameters 
% 	vtol.Gamma  = vtol.Jx*vtol.Jz-vtol.Jxz^2;
% 	vtol.Gamma1 = (vtol.Jxz*(vtol.Jx-vtol.Jy+vtol.Jz))/vtol.Gamma;
% 	vtol.Gamma2 = (vtol.Jz*(vtol.Jz-vtol.Jy)+vtol.Jxz*vtol.Jxz)/vtol.Gamma;
% 	vtol.Gamma3 = vtol.Jz/vtol.Gamma;
% 	vtol.Gamma4 = vtol.Jxz/vtol.Gamma;
% 	vtol.Gamma5 = (vtol.Jz-vtol.Jx)/vtol.Jy;
% 	vtol.Gamma6 = vtol.Jxz/vtol.Jy;
% 	vtol.Gamma7 = (vtol.Jx*(vtol.Jx-vtol.Jy)+vtol.Jxz*vtol.Jxz)/vtol.Gamma;
% 	vtol.Gamma8 = vtol.Jx/vtol.Gamma;
% 
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %  Appendix-E coefficients
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 	% aerodynamic coefficients
% % 	vtol.C_L_0         = 0.28;
% % 	vtol.C_D_0         = 0.03;
% % 	vtol.C_m_0         = -0.02338;
% % 	vtol.C_L_alpha     = 3.45;
% % 	vtol.C_D_alpha     = 0.30;
% % 	vtol.C_m_alpha     = -0.38;
% % 	vtol.C_L_q         = 0;
% % 	vtol.C_D_q         = 0.0;
% % 	vtol.C_m_q         = -3.6;
% % 	vtol.C_L_delta_e   = -0.36;
% % 	vtol.C_D_delta_e   = 0.0;
% % 	vtol.C_m_delta_e   = -0.5;
% % 	vtol.M             = 50;
% % 	vtol.alpha0        = 0.4712;
% % 	vtol.epsilon       = 0.16;
% % 	vtol.C_D_p         = 0.0437;
% % 
% % 	vtol.C_Y_0         = 0.0;
% % 	vtol.C_ell_0       = 0.0;
% % 	vtol.C_n_0         = 0.0;
% % 	vtol.C_Y_beta      = -0.98;
% % 	vtol.C_ell_beta    = -0.12;
% % 	vtol.C_n_beta      = 0.25;
% % 	vtol.C_Y_p         = 0.0;
% % 	vtol.C_ell_p       = -0.26;
% % 	vtol.C_n_p         = 0.022;
% % 	vtol.C_Y_r         = 0.0;
% % 	vtol.C_ell_r       = 0.14;
% % 	vtol.C_n_r         = -0.35;
% % 	vtol.C_Y_delta_a   = 0.0;
% % 	vtol.C_ell_delta_a = 0.08;
% % 	vtol.C_n_delta_a   = 0.06;
% % 	vtol.C_Y_delta_r   = -0.17;
% % 	vtol.C_ell_delta_r = 0.105;
% % 	vtol.C_n_delta_r   = -0.032;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Errata coefficients (mostly same as given in the website)
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aerodynamic coefficients
CLo        = 0.23;
Cdo         = 0.043;
Cmo         = 0.0135;
CL_alpha    = 5.61;
Cd_alpha     = 0.030;
Cm_alpha     = -2.74;
CL_q         = 7.95;
Cd_q         = 0.0;
% vtol.C_m_q         = -38.21;
CL_delta_e   = 0.13;
Cd_delta_e   = 0.0135;
Cm_delta_e   = -0.99;
% vtol.M             = 50;
% vtol.alpha0        = 0.47;
% vtol.epsilon       = 0.16;
% vtol.C_D_p         = 0.0;

CYo         = 0.0;
Clo       = 0.0;
Cno         = 0.0;
%vtol.C_Y_beta      = -0.98;
CY_beta      = -0.83; % corrected from Errata
Cl_beta    = -0.13;
Cn_beta      = 0.073;
% vtol.C_Y_p         = 0.0;
Cl_p       = -0.51;
Cn_p         = -0.069;
% vtol.C_Y_r         = 0.0;
Cl_r       = 0.25;
Cn_r         = -0.095;
CY_delta_a   = 0.075;
Cl_delta_a = 0.17;
Cn_delta_a   = -0.011;
CY_delta_r   = 0.19;
Cl_delta_r = 0.0024;
Cn_delta_r   = -0.069;

%  =vtol.C_L_0; =aero_data(1,2);=aero_data(1,3); =aero_data(1,4); =aero_data(1,5); CL_delta_r=aero_data(1,6); CL_p=aero_data(1,7);=aero_data(1,8);CL_r=aero_data(1,9);
% 
%  =aero_data(2,1); =aero_data(2,2);=aero_data(2,3); =aero_data(2,4); Cd_delta_a=aero_data(2,5); Cd_delta_r=aero_data(2,6); Cd_p=aero_data(2,7);=aero_data(2,8);Cd_r=aero_data(2,9);
% 
%  =aero_data(3,1); CY_alpha=aero_data(3,2);=aero_data(3,3); =aero_data(3,4); =aero_data(3,5); =aero_data(3,6); CY_p=aero_data(3,7);CY_q=aero_data(3,8);CY_r=aero_data(3,9);
% 
%  =aero_data(4,1); Cl_alpha=aero_data(4,2);=aero_data(4,3); Cl_delta_e=aero_data(4,4); =aero_data(4,5); =aero_data(4,6); =aero_data(4,7);Cl_q=aero_data(4,8);Cl_r=aero_data(4,9);
% 
% Cmo =aero_data(5,1); Cm_alpha=aero_data(5,2);Cm_beta=aero_data(5,3); =aero_data(5,4); Cm_delta_a=aero_data(5,5); Cm_delta_r=aero_data(5,6); Cm_p=aero_data(5,7);Cm_q=aero_data(5,8);Cm_r=aero_data(5,9);
% 
%  =aero_data(6,1); Cn_alpha=aero_data(6,2);=aero_data(6,3); Cn_delta_e=aero_data(6,4); =aero_data(6,5); =aero_data(6,6); =aero_data(6,7);Cn_q=aero_data(6,8);Cn_r=aero_data(6,9);




g=9.81;



tb=[(mass*g/(0.5*rho*V*V*s)) - CLo;-Cmo];
ta=[CL_alpha,CL_delta_e;Cm_alpha,Cm_delta_e];

trim_set_long = inv(ta)*tb;

alpha_trim = trim_set_long(1);
delta_e_trim = trim_set_long(2);

% CYo + CY_beta*beta + CY_delta_a*delta_a + CY_delta_r*delta_r;
% Clo + Cl_beta*beta + Cl_delta_a*delta_a + Cl_delta_r*delta_r;
% Cno + Cn_beta*beta + Cn_delta_a*delta_a + Cn_delta_r*delta_r;

tb1=[-CYo;-Clo;-Cno];
ta1=[CY_beta,CY_delta_a,CY_delta_r;Cl_beta,Cl_delta_a,Cl_delta_r;Cn_beta,Cn_delta_a,Cn_delta_r];

trim_set_lat=inv(ta1)*tb1;

beta_trim = trim_set_lat(1);
delta_a_trim = trim_set_lat(2);
delta_r_trim = trim_set_lat(3);


% drag = 1/2 *rho *V*V*s*( Cdo + Cd_alpha*alpha + Cd_delta_e*delta_e );
Tx_trim =  0.5*rho*V*V*s*( Cdo + Cd_alpha*alpha_trim + Cd_delta_e*delta_e_trim );

stall_speed=sqrt((mass*g)/(0.5*1.225*s*(CLo+CL_alpha*(12.0/57.3))));
