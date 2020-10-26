    % initialize the VTOL viewer

% compute trim conditions using 'vtolsim_trim.slx'
% nominal airspeed vtol.Va0 specified above with aircraft parameters
    vtol.Va0 = 35;
    gamma = 15*pi/180;   % desired flight path angle (radians)
%     gamma = 0;
    R     = Inf;        % desired radius (m) - use (+) for right handed orbit, 
                        %                          (-) for left handed orbit 
                        
    %autopilot sample rate
    vtol.Ts = 0.01;
    %vtol.Ts = 1;
    
    vtol.pn0    = 0;     % initial North position
	vtol.pe0    = 0;     % initial East position
	vtol.pd0    = 0;  % initial Down position (negative altitude)
	vtol.u0     = vtol.Va0;     % initial velocity along body x-axis
	vtol.v0     = 0;     % initial velocity along body y-axis
	vtol.w0     = 0;     % initial velocity along body z-axis
	vtol.phi0   = 0;     % initial roll angle
	vtol.theta0 = 0;     % initial pitch angle
	vtol.psi0   = 0;     % initial yaw angle
% 	e = Euler2Quaternion(vtol.phi0,vtol.theta0,vtol.psi0);
% 	vtol.e0     = e(1);  % initial quaternion
% 	vtol.e1     = e(2);
% 	vtol.e2     = e(3);
% 	vtol.e3     = e(4);
	vtol.p0     = 0;     % initial body frame roll rate
	vtol.q0     = 0;     % initial body frame pitch rate
	vtol.r0     = 0;     % initial body frame yaw rate
    
    %physical parameters of airframe
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Appendix-E coefficients
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%1
	vtol.gravity = 9.81;
    vtol.mass = 11.0;
	vtol.Jx   = 0.8244;
	vtol.Jy   = 1.135;
	vtol.Jz   = 1.759;
	vtol.Jxz  = 0.1204;
	vtol.S_wing        = 0.55;
	vtol.b             = 2.90;
	vtol.c             = 0.19;
% 	vtol.mass = 2477.0;
%     vtol.Jx   = 2000;
% 	vtol.Jy   = 2000;
% 	vtol.Jz   = 2500;
% 	vtol.Jxz  = 0;
% 	vtol.S_wing        = 21.55;
% 	vtol.b             = 21.90;
% 	vtol.c             = 2.19;
    
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

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Appendix-E coefficients
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% aerodynamic coefficients
% 	vtol.C_L_0         = 0.28;
% 	vtol.C_D_0         = 0.03;
% 	vtol.C_m_0         = -0.02338;
% 	vtol.C_L_alpha     = 3.45;
% 	vtol.C_D_alpha     = 0.30;
% 	vtol.C_m_alpha     = -0.38;
% 	vtol.C_L_q         = 0;
% 	vtol.C_D_q         = 0.0;
% 	vtol.C_m_q         = -3.6;
% 	vtol.C_L_delta_e   = -0.36;
% 	vtol.C_D_delta_e   = 0.0;
% 	vtol.C_m_delta_e   = -0.5;
% 	vtol.M             = 50;
% 	vtol.alpha0        = 0.4712;
% 	vtol.epsilon       = 0.16;
% 	vtol.C_D_p         = 0.0437;
% 
% 	vtol.C_Y_0         = 0.0;
% 	vtol.C_ell_0       = 0.0;
% 	vtol.C_n_0         = 0.0;
% 	vtol.C_Y_beta      = -0.98;
% 	vtol.C_ell_beta    = -0.12;
% 	vtol.C_n_beta      = 0.25;
% 	vtol.C_Y_p         = 0.0;
% 	vtol.C_ell_p       = -0.26;
% 	vtol.C_n_p         = 0.022;
% 	vtol.C_Y_r         = 0.0;
% 	vtol.C_ell_r       = 0.14;
% 	vtol.C_n_r         = -0.35;
% 	vtol.C_Y_delta_a   = 0.0;
% 	vtol.C_ell_delta_a = 0.08;
% 	vtol.C_n_delta_a   = 0.06;
% 	vtol.C_Y_delta_r   = -0.17;
% 	vtol.C_ell_delta_r = 0.105;
% 	vtol.C_n_delta_r   = -0.032;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Errata coefficients (mostly same as given in the website)
% 	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aerodynamic coefficients
vtol.C_L_0         = 0.23;
vtol.C_D_0         = 0.043;
vtol.C_m_0         = 0.0135;
vtol.C_L_alpha     = 5.61;
vtol.C_D_alpha     = 0.030;
vtol.C_m_alpha     = -2.74;
vtol.C_L_q         = 7.95;
vtol.C_D_q         = 0.0;
vtol.C_m_q         = -38.21;
vtol.C_L_delta_e   = 0.13;
vtol.C_D_delta_e   = 0.0135;

vtol.M             = 50;
vtol.alpha0        = 0.47;
vtol.epsilon       = 0.16;
vtol.C_D_p         = 0.0;

vtol.C_Y_0         = 0.0;
vtol.C_ell_0       = 0.0;
vtol.C_n_0         = 0.0;
%vtol.C_Y_beta      = -0.98;
vtol.C_Y_beta      = -0.83; % corrected from Errata
vtol.C_ell_beta    = -0.13;
vtol.C_n_beta      = 0.073;
vtol.C_Y_p         = 0.0;
vtol.C_ell_p       = -0.51;
vtol.C_n_p         = -0.069;
vtol.C_Y_r         = 0.0;
vtol.C_ell_r       = 0.25;
vtol.C_n_r         = -0.095;
vtol.C_Y_delta_a   = 0.075;

% vtol.C_ell_delta_a = 0.17; % Such high value causing aileron saturation
% at 35 m/s. So reduced the value to 0.1 assuming that 0.17 is wrong for this cruise velocity.
vtol.C_ell_delta_a = 0.06;
% vtol.C_n_delta_r   = -0.069;
vtol.C_n_delta_r   = -0.069;
% vtol.C_m_delta_e   = -0.99;
vtol.C_m_delta_e   = -0.99;

vtol.C_n_delta_a   = -0.011;
vtol.C_Y_delta_r   = 0.19;
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

%     [x_trim,u_trim,y_trim,dx_trim] = compute_trim('vtolsim_trim', vtol.Va0, gamma, R);
% x_trim = ;
% y_trim = ;
% u_trim = ;
% x_trim = [-6.49412735651667e-19;-1.89023386605556e-18;-7.08801968609821e-21;24.9715506011084;0;1.19233408752810;-0.00105283494365042;-0.214087951499671;-8.82589263945641e-20;5.93526532702691e-22;-1.26934759908545e-21;1.86059633830992e-21];
% y_trim = [25.0000000000000;0.0477114630404312;0];
% u_trim = [-0.118413544172506;0.0113916266907877;-0.00181605642896615;0.390555072059526];
% x_trim = [-3.12801987404505e-20;-1.38411186942073e-19;-6.18634078607259e-22;44.9942259979256;-2.43772173372454e-26;-0.720851474019738;0.000805942704631727;-0.277818989435158;-1.95283233776795e-19;-4.43640103716278e-24;2.46656575241327e-23;-6.64999383418818e-25];
% y_trim = [45;-0.0160196068155070;-5.41715940827675e-28];
% u_trim = [0.0579734572469589;-0.00264870725133006;0.000422257677748270;1.22347014383771];
% x_trim = [-2.44983631910445e-21;1.63879088259825e-20;1.68808157122805e-22;34.9999046341372;-8.41794242333320e-27;0.0817043530336400;-0.000218533898618586;-0.259464975648029;6.33157202970083e-21;-1.30411612842414e-24;-5.12443247328518e-25;1.33967619123344e-24];
% y_trim = [35;0.00233441220689709;-2.40512640666663e-28];
% u_trim = [0.00717546520515350;0.00119325168647462;-0.000190228529727838;0.850374370516102];
x_trim = [-6.89803428243025e-22;7.12872273378489e-21;6.14736666979624e-23;34.9997252003585;-8.50490663441403e-30;0.138693544868103;0.000389714048294023;0.00396268278049968;-1.72693704719555e-22;5.55365203015969e-24;-6.82589055765580e-25;8.10069513911015e-24];
y_trim = [35;0.00396268308141684;-2.42997332411830e-31];
u_trim = [0.00266893773426046;-0.00220161676714189;0.000350982383167547;0.939015268042139];


    vtol.x_trim = x_trim;
    vtol.u_trim = u_trim;
    
    % set initial conditions to trim conditions
    % initial conditions
    vtol.pn0    = 0;           % initial North position
    vtol.pe0    = 0;           % initial East position
    vtol.pd0    = 0;   
    vtol.u0     = x_trim(4);   % initial velocity along body x-axis
    vtol.v0     = x_trim(5);   % initial velocity along body y-axis
    vtol.w0     = x_trim(6);   % initial velocity along body z-axis
    vtol.phi0   = x_trim(7);   % initial roll angle
    vtol.theta0 = x_trim(8);   % initial pitch angle
    vtol.psi0   = x_trim(9);   % initial yaw angle
    vtol.p0     = x_trim(10);  % initial body frame roll rate
    vtol.q0     = x_trim(11);  % initial body frame pitch rate
    vtol.r0     = x_trim(12);  % initial body frame yaw rate
    
    vtol.Va_trim     = sqrt(x_trim(4)^2 + x_trim(5)^2 + x_trim(6)^2); % In the absence of wind, Vg = Va.
  
    %    compute different transfer functions 
     [T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta, T_v_delta_r]...
      = compute_tf_model(x_trim, u_trim, y_trim, vtol);
  
     trans_func = [T_phi_delta_a T_chi_phi T_theta_delta_e T_h_theta T_h_Va T_Va_delta_t T_Va_theta T_v_delta_r];
     
%    linearize the equations of motion around trim conditions
     [A_lon, B_lon, A_lat, B_lat] = compute_ss_model('vtolsim_trim', x_trim, u_trim);
     
     [AP] = compute_autopilot_gains(trans_func, A_lon, B_lon, A_lat, B_lat, vtol);
     

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

AP.phi_max = 45*pi/180;

atp.Va0 = 35;

% number of waypoints in data structure
atp.size_waypoint_array = 100;
atp.R_min = vtol.Va0^2/vtol.gravity/tan(AP.phi_max);

% create random city map
city_width      = 2000;  % city size 
building_height = 300;   % maximum height
num_blocks      = 5;    % number of blocks in city
street_width    = 1;   % percent of block that is street.

atp.map = createWorld(city_width, building_height, num_blocks, street_width);



     
     
