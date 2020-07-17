    vtol.Va0 = 25;
    gamma = 0*pi/180;   % desired flight path angle (radians)
    R     = Inf;        % desired radius (m) - use (+) for right handed orbit, 
                   
    %autopilot sample rate
    vtol.Ts = 0.01;

    vtol.pn0    = 0;     % initial North position
	vtol.pe0    = 0;     % initial East position
	vtol.pd0    = 0;  % initial Down position (negative altitude)
	vtol.u0     = vtol.Va0;     % initial velocity along body x-axis
	vtol.v0     = 0;     % initial velocity along body y-axis
	vtol.w0     = 0;     % initial velocity along body z-axis
	vtol.phi0   = 0;     % initial roll angle
	vtol.theta0 = 0;     % initial pitch angle
	vtol.psi0   = 0;     % initial yaw angle
	vtol.p0     = 0;     % initial body frame roll rate
	vtol.q0     = 0;     % initial body frame pitch rate
	vtol.r0     = 0;     % initial body frame yaw rate
    
    %physical parameters of airframe
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Appendix-E coefficients
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%1
	vtol.gravity = 9.81;
%     vtol.mass = 11.0;
% 	vtol.Jx   = 0.8244;
% 	vtol.Jy   = 1.135;
% 	vtol.Jz   = 1.759;
% 	vtol.Jxz  = 0.1204;
% 	vtol.S_wing        = 0.55;
% 	vtol.b             = 2.90;
% 	vtol.c             = 0.19;
% 	vtol.mass = 2477.0;
%     vtol.Jx   = 2000;
% 	vtol.Jy   = 2000;
% 	vtol.Jz   = 2500;
% 	vtol.Jxz  = 0;
% 	vtol.S_wing        = 21.55;
% 	vtol.b             = 21.90;
% 	vtol.c             = 2.19;
vtol.mass = 22;
vtol.Jx   = 1.5;
vtol.Jy   = 1.7;
vtol.Jz   = 2.1;
vtol.Jxz  = 0.15;
% aerodynamic coefficients
vtol.S_wing        = 0.79;
vtol.b             = 3.3;
vtol.c             = 0.19;
   
	vtol.S_prop        = 0.2027;
	vtol.rho           = 1.2682;
	vtol.e             = 0.9;
	vtol.AR            = vtol.b^2/vtol.S_wing;
    
 	% Gamma parameters 
	vtol.Gamma  = vtol.Jx*vtol.Jz-vtol.Jxz^2;
	vtol.Gamma1 = (vtol.Jxz*(vtol.Jx-vtol.Jy+vtol.Jz))/vtol.Gamma;
	vtol.Gamma2 = (vtol.Jz*(vtol.Jz-vtol.Jy)+vtol.Jxz*vtol.Jxz)/vtol.Gamma;
	vtol.Gamma3 = vtol.Jz/vtol.Gamma;
	vtol.Gamma4 = vtol.Jxz/vtol.Gamma;
	vtol.Gamma5 = (vtol.Jz-vtol.Jx)/vtol.Jy;
	vtol.Gamma6 = vtol.Jxz/vtol.Jy;
	vtol.Gamma7 = (vtol.Jx*(vtol.Jx-vtol.Jy)+vtol.Jxz*vtol.Jxz)/vtol.Gamma;
	vtol.Gamma8 = vtol.Jx/vtol.Gamma;

% aerodynamic coefficients
vtol.C_L_0         = 0.23;
vtol.C_D_0         = 0.043;
vtol.C_m_0         = 0.0135;
vtol.C_L_alpha     = 5.61;
% vtol.C_D_alpha     = 0.030;
vtol.C_D_alpha     = 0.3;
vtol.C_m_alpha     = -2.74;
vtol.C_L_q         = 7.95;
vtol.C_D_q         = 0.0;
vtol.C_m_q         = -38.21;
vtol.C_L_delta_e   = 0.13;
vtol.C_D_delta_e   = 0.0135;

vtol.M             = 50;
vtol.alpha0        = 0.47;
vtol.epsilon       = 0.16;
% vtol.C_D_p         = 0.0;
vtol.C_D_p         = 0.0437;

% vtol.C_Y_0         = 0.0;
% vtol.C_ell_0       = 0.0;
% vtol.C_n_0         = 0.0;
% %vtol.C_Y_beta      = -0.98;
% vtol.C_Y_beta      = -0.83; % corrected from Errata
% vtol.C_ell_beta    = -0.13;
% vtol.C_n_beta      = 0.073;
% vtol.C_Y_p         = 0.0;
% vtol.C_ell_p       = -0.51;
% vtol.C_n_p         = -0.069;
% vtol.C_Y_r         = 0.0;
% vtol.C_ell_r       = 0.25;
% vtol.C_n_r         = -0.095;
% vtol.C_Y_delta_a   = 0.075;

vtol.C_Y_0         = 0.0;
vtol.C_ell_0       = 0.0;
vtol.C_n_0         = 0.0;
vtol.C_Y_beta      = -0.83;
vtol.C_ell_beta    = -0.13;
vtol.C_n_beta      = 0.0726;
vtol.C_Y_p         = 0.0;
vtol.C_ell_p       = -0.6;
vtol.C_n_p         = -0.12;
vtol.C_Y_r         = 0.0;
vtol.C_ell_r       = 0.2;
vtol.C_n_r         = -0.2;
vtol.C_Y_delta_a   = 0;

% % vtol.C_ell_delta_a = 0.17; % Such high value causing aileron saturation
% % at 35 m/s. So reduced the value to 0.1 assuming that 0.17 is wrong for this cruise velocity.
% vtol.C_ell_delta_a = 0.1;
% % vtol.C_n_delta_r   = -0.069;
% vtol.C_n_delta_r   = -0.069;
% % vtol.C_m_delta_e   = -0.99;
% vtol.C_m_delta_e   = -0.99;

vtol.C_ell_delta_a = -0.1695;
vtol.C_n_delta_r   = -0.0693;
vtol.C_m_delta_e   = -0.9918;

% vtol.C_n_delta_a   = -0.011;
% vtol.C_Y_delta_r   = 0.19;
% vtol.C_ell_delta_r = 0.0024;

vtol.C_n_delta_a   = 0.0108;
vtol.C_Y_delta_r   = 0.1914;
vtol.C_ell_delta_r = 0.0024;

% vtol.C_ell_beta    = -0.013;
% vtol.C_ell_p       = -0.051;
% vtol.C_ell_r       = 0.025;
% vtol.C_ell_delta_a = 0.17;
% vtol.C_ell_delta_r = 0.0024;

	% Parameters for propulsion thrust and torque models
	vtol.D_prop = 0.508;     % prop diameter in m

	% Motor parameters
	vtol.K_V = 145;                    % from datasheet RPM/V
	%vtol.KQ = (1/vtol.K_V)*60/(2*pi);   % KQ in N-m/A, V-s/rad
    vtol.KQ = 0.0659; % Corrected using Errata
	vtol.R_motor = 0.042;              % ohms
	vtol.i0 = 1.5;                     % no-load (zero-torque) current (A)

	% Inputs
	vtol.ncells = 12;
	vtol.V_max = 3.7*vtol.ncells;       % max voltage for specified number of battery cells

	% Coeffiecients from prop_data fit
	vtol.C_Q2 = -0.01664;
	vtol.C_Q1 = 0.004970;
	vtol.C_Q0 = 0.005230;

	vtol.C_T2 = -0.1079;
	vtol.C_T1 = -0.06044;
	vtol.C_T0 = 0.09357;
    %vtol.C_T0 = 1.1;
        
    [x_trim,u_trim,y_trim,dx_trim] = compute_trim('vtolsim_trim', vtol.Va0, gamma, R);
%     vtol.x_trim = x_trim;
%     vtol.u_trim = u_trim;
%     
%     % set initial conditions to trim conditions
%     % initial conditions
%     vtol.pn0    = 0;           % initial North position
%     vtol.pe0    = 0;           % initial East position
%     vtol.pd0    = 0;   
%     vtol.u0     = x_trim(4);   % initial velocity along body x-axis
%     vtol.v0     = x_trim(5);   % initial velocity along body y-axis
%     vtol.w0     = x_trim(6);   % initial velocity along body z-axis
%     vtol.phi0   = x_trim(7);   % initial roll angle
%     vtol.theta0 = x_trim(8);   % initial pitch angle
%     vtol.psi0   = x_trim(9);   % initial yaw angle
%     vtol.p0     = x_trim(10);  % initial body frame roll rate
%     vtol.q0     = x_trim(11);  % initial body frame pitch rate
%     vtol.r0     = x_trim(12);  % initial body frame yaw rate
%     
%     vtol.Va_trim     = sqrt(x_trim(4)^2 + x_trim(5)^2 + x_trim(6)^2); % In the absence of wind, Vg = Va.
%   
%     
% %    compute different transfer functions 
%      [T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta, T_v_delta_r]...
%       = compute_tf_model(x_trim, u_trim, y_trim, vtol);
%   
%      trans_func = [T_phi_delta_a T_chi_phi T_theta_delta_e T_h_theta T_h_Va T_Va_delta_t T_Va_theta T_v_delta_r];
%      
% %    linearize the equations of motion around trim conditions
%      [A_lon, B_lon, A_lat, B_lat] = compute_ss_model('vtolsim_trim', x_trim, u_trim);
%      
%      [AP] = compute_autopilot_gains(trans_func, A_lon, B_lon, A_lat, B_lat, vtol);
%      


     
     
