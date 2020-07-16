P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for C-Tail UAV
%physical parameters of airframe
P.mass = 22;
P.Jx   = 1.5;
P.Jy   = 1.7;
P.Jz   = 2.1;
P.Jxz  = 0.15;
% aerodynamic coefficients
P.S_wing        = 0.79;
P.b             = 3.3;
P.c             = 0.19;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 60;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.23;
P.C_L_alpha     = 5.6106;
P.C_L_q         = 7.9543;
P.C_L_delta_e   = 0.13;
P.C_D_0         = 0.0434;
P.C_D_alpha     = 0.3;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0135;
P.C_m_0         = 0.135;
P.C_m_alpha     = -2.7397;
P.C_m_q         = -38.2067;
P.C_m_delta_e   = -0.9918;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.83;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0;
P.C_Y_delta_r   = 0.1914;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.13;
P.C_ell_p       = -0.6;
P.C_ell_r       = 0.2;
P.C_ell_delta_a = -0.1695;
P.C_ell_delta_r = 0.0024;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.0726;
P.C_n_p         = -0.12;
P.C_n_r         = -0.2;
P.C_n_delta_a   = 0.0108;
P.C_n_delta_r   = -0.0693;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% wind parameters
P.wind_n = 0;
P.wind_e = 0;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;



% initial airspeed
P.Va0 = 25;%17;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;         % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
P.Ts = 0.01;
% gps sample rate
P.Ts_gps=1;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate


% run trim commands
[x_trim,u_trim,y_trim,dx_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

%-------------------Used In AutoPilot------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  P.tau    = P.Jx*P.Jz-(P.Jxz*P.Jxz);
  P.tau1   = P.Jxz*(P.Jx-P.Jy+P.Jz)/P.tau;
  P.tau2   = (P.Jz*(P.Jz-P.Jy)+P.Jxz*P.Jxz)/P.tau;
  P.tau3   = P.Jz/P.tau;
  P.tau4   = P.Jxz/P.tau;
  P.tau5   = (P.Jz-P.Jx)/P.Jy;
  P.tau6   = P.Jxz/P.Jy;
  P.tau7   = ((P.Jx-P.Jy)*P.Jx+P.Jxz*P.Jxz)/P.tau;
  P.tau8   = P.Jx/P.tau;
  %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Trim Constants~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    P.Va_trim      = sqrt(x_trim(4)*x_trim(4)+x_trim(5)*x_trim(5)+x_trim(6)*x_trim(6));
    P.alpha_trim   = atan(x_trim(6)/x_trim(4));%0;
    P.beta_trim    = asin(x_trim(5)/P.Va_trim);%0;
    theta_trim     = x_trim(8);
    delta_e_trim   = u_trim(1);
    delta_t_trim   = u_trim(4); 
%------------------- Roll Angle-----------------------%
P.CPp        = P.tau3*P.C_ell_p+P.tau4*P.C_n_p;
P.CP_delta_a = P.tau3*P.C_ell_delta_a+P.tau4*P.C_n_delta_a;
P.a_phi1     = -0.5*P.rho*P.Va_trim*P.Va_trim*P.S_wing*P.b*P.CPp*P.b/2/P.Va_trim;
P.a_phi2     = 0.5*P.rho*P.Va_trim*P.Va_trim*P.S_wing*P.b*P.CP_delta_a;
% 
% %-----------------Course and Heading------------------%
% P.a_beta1    = -P.rho*P.Va0*P.S_wing*P.C_Y_beta/2/P.mass;
% P.a_beta2    = P.rho*P.Va0*P.S_wing*P.C_Y_beta/2/P.mass;
%-----------------Pitch Angle-------------------------%
P.a_theta1    = -P.rho*P.Va_trim*P.Va_trim*P.c*P.S_wing*P.C_m_q*P.c/4/P.Jy/P.Va_trim;
P.a_theta2    = -P.rho*P.Va_trim*P.Va_trim*P.c*P.S_wing*P.C_m_alpha/2/P.Jy;
P.a_theta3    =  P.rho*P.Va_trim*P.Va_trim*P.c*P.S_wing*P.C_m_delta_e/2/P.Jy;

%---------------------Airspeed------------------------%
P.a_V1        = (P.rho*P.Va_trim*P.S_wing*(P.C_D_0+P.C_D_alpha*P.alpha_trim+P.C_D_delta_e*delta_e_trim)+P.rho*P.S_prop*P.C_prop*P.Va_trim)/P.mass;
P.a_V2        = P.rho*P.S_prop*P.C_prop*P.k_motor*P.k_motor*delta_t_trim/P.mass;
P.a_V3        = P.gravity*cos(theta_trim-P.alpha_trim);

%---------------------Sideslip------------------------%
P.a_beta1     = -P.rho*P.Va_trim*P.S_wing*P.C_Y_beta/2/P.mass;
P.a_beta2     = P.rho*P.Va_trim*P.S_wing*P.C_Y_delta_r/2/P.mass;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%roll
P.delta_a_max=20*pi/180;
P.e_phi_max=15*pi/180;

P.zeta_phi=0.65;%0.7*5;


P.omega_n_phi = sqrt(abs(P.a_phi2)*P.delta_a_max*sqrt(1-P.zeta_phi^2)/P.e_phi_max);
    
    % set control gains based on zeta and wn
    P.kp_phi = P.omega_n_phi^2/P.a_phi2;
    P.kd_phi = (2*P.zeta_phi*P.omega_n_phi - P.a_phi1)/P.a_phi2;
    P.ki_phi = 0.1;
    
%Beta
P.delta_r_max=20*pi/180;
P.e_beta_max=15*pi/180;

P.zeta_beta=.707;
P.beta_kp=(P.delta_r_max/P.e_beta_max)*sign(P.a_beta2);
P.omega_n_beta=(P.a_beta2*P.beta_kp+P.a_beta1)/2/P.zeta_beta;
P.beta_ki=P.omega_n_beta^2/P.a_beta2;

%course
P.W_x=15; %bandwidth separation for roll and course
% P.W_x=15; %bandwidth separation for roll and course
P.omega_n_chi=P.omega_n_phi/P.W_x;
P.zeta_chi=1.2;%.707;
P.kp_chi=0.5*P.zeta_chi*P.omega_n_chi*P.Va0/P.gravity;
P.ki_chi=P.omega_n_chi^2*P.Va0/P.gravity;
P.kd_chi=0.5;

%pitch
P.delta_e_max=20*pi/180;
P.e_theta_max=10*pi/180;
P.kp_theta=2*(P.delta_e_max/P.e_theta_max)*sign(P.a_theta3);
P.omega_n_theta=sqrt(P.a_theta2+(P.delta_e_max/P.e_theta_max)*abs(P.a_theta3));
P.zeta_theta=0.707;
P.kd_theta=(2*P.zeta_theta*P.omega_n_theta-P.a_theta1)/P.a_theta3;
P.K_theta_DC=(P.kp_theta*P.a_theta3)/(P.a_theta2+P.kp_theta*P.a_theta3);
P.ki_theta=0.1;

% altitude
P.W_h=12; %bandwidth separation pitch and altitude
P.omega_n_h=P.omega_n_theta/P.W_h;
P.zeta_h=0.707;
P.ki_h=P.omega_n_h^2/(P.K_theta_DC*P.Va0);
P.kp_h=(2*P.zeta_h*P.omega_n_h)/(P.K_theta_DC*P.Va0);
P.kd_h=0;

% airspeed with Pitch
P.W_V2=15; % bandwidth separation
%P.W_V2=16; % bandwidth separation
P.omega_n_V2=P.omega_n_theta/P.W_V2;
P.zeta_V2=.707;
%P.zeta_V2=.707*8;

P.ki_V2=-P.omega_n_V2^2/(P.K_theta_DC*P.gravity);
P.kp_V2=(P.a_V1-2*P.zeta_V2*P.omega_n_V2)/(P.K_theta_DC*P.gravity);
P.kd_V2=0.1;

% airspeed with throttle
P.omega_n_V=P.omega_n_V2;
P.zeta_V=.707*2;
P.ki_V=P.omega_n_V^2/P.a_V2;
P.kp_V=5*(2*P.zeta_V*P.omega_n_V-P.a_V1)/P.a_V2;
P.kd_V=0.1;



P.altitude_take_off_zone=20;
P.altitude_hold_zone=30;
P.climb_pitch=20*pi/180;
P.theta_max=30*pi/180;
P.phi_max=30*pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% number of waypoints in data structure
P.size_waypoint_array = 100;
P.R_min = P.Va0^2/P.gravity/tan(P.phi_max);

% create random city map
city_width      = 2000;  % city size 
building_height = 300;   % maximum height
num_blocks      = 5;    % number of blocks in city
street_width    = 1;   % percent of block that is street.

P.map = createWorld(city_width, building_height, num_blocks, street_width);

