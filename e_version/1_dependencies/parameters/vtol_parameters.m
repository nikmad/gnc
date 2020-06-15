% initialize the vtol viewer
addpath('../1_dependencies/tools');  

% initial conditions
vtol.pn0    = 0;     % initial North position
vtol.pe0    = 0;     % initial East position
vtol.pd0    = -100;  % initial Down position (negative altitude)
vtol.u0     = 25;     % initial velocity along body x-axis
vtol.v0     = 0;     % initial velocity along body y-axis
vtol.w0     = 0;     % initial velocity along body z-axis
vtol.phi0   = 0;     % initial roll angle
vtol.theta0 = 0;     % initial pitch angle
vtol.psi0   = 0;     % initial yaw angle
e = Euler2Quaternion(vtol.phi0, vtol.theta0, vtol.psi0);
vtol.e0     = e(1);  % initial quaternion
vtol.e1     = e(2);
vtol.e2     = e(3);
vtol.e3     = e(4);
vtol.p0     = 0;     % initial body frame roll rate
vtol.q0     = 0;     % initial body frame pitch rate
vtol.r0     = 0;     % initial body frame yaw rate
   
%physical parameters of airframe
vtol.gravity = 9.81;
vtol.mass = 11.0;
vtol.Jx   = 0.824;
vtol.Jy   = 1.135;
vtol.Jz   = 1.759;
vtol.Jxz  = 0.120;
vtol.S_wing        = 0.55;
vtol.b             = 2.90;
vtol.c             = 0.19;
vtol.S_prop        = 0.2027;
vtol.rho           = 1.2682;
vtol.e             = 0.9;
vtol.AR            = vtol.b^2/vtol.S_wing;

% Gamma parameters from uavbook page 36
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
vtol.C_m_delta_e   = -0.99;
vtol.M             = 50;
vtol.alpha0        = 0.47;
vtol.epsilon       = 0.16;
vtol.C_D_p         = 0.0;

vtol.C_Y_0         = 0.0;
vtol.C_ell_0       = 0.0;
vtol.C_n_0         = 0.0;
vtol.C_Y_beta      = -0.98;
vtol.C_ell_beta    = -0.13;
vtol.C_n_beta      = 0.073;
vtol.C_Y_p         = 0.0;
vtol.C_ell_p       = -0.51;
vtol.C_n_p         = -0.069;
vtol.C_Y_r         = 0.0;
vtol.C_ell_r       = 0.25;
vtol.C_n_r         = -0.095;
vtol.C_Y_delta_a   = 0.075;
vtol.C_ell_delta_a = 0.17;
vtol.C_n_delta_a   = -0.011;
vtol.C_Y_delta_r   = 0.19;
vtol.C_ell_delta_r = 0.0024;
vtol.C_n_delta_r   = -0.069;

% Parameters for propulsion thrust and torque models
vtol.D_prop = 0.508;     % prop diameter in m

% Motor parameters
vtol.K_V = 145;                    % from datasheet RPM/V
vtol.KQ = (1/vtol.K_V)*60/(2*pi);   % KQ in N-m/A, V-s/rad
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



