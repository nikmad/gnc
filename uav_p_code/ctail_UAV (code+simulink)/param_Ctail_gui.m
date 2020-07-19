% initial airspeed
vtol.Va0 = 25;%17;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;         % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
vtol.Ts = 0.01;
% gps sample rate
vtol.Ts_gps=1;

% first cut at initial conditions
vtol.pn0    = 0;  % initial North position
vtol.pe0    = 0;  % initial East position
vtol.pd0    = 0;  % initial Down position (negative altitude)
vtol.u0     = vtol.Va0; % initial velocity along body x-axis
vtol.v0     = 0;  % initial velocity along body y-axis
vtol.w0     = 0;  % initial velocity along body z-axis
vtol.phi0   = 0;  % initial roll angle
vtol.theta0 = 0;  % initial pitch angle
vtol.psi0   = 0;  % initial yaw angle
vtol.p0     = 0;  % initial body frame roll rate
vtol.q0     = 0;  % initial body frame pitch rate
vtol.r0     = 0;  % initial body frame yaw rate

vtol.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for C-Tail UAV
%physical parameters of airframe
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

vtol.Gamma    = vtol.Jx*vtol.Jz-(vtol.Jxz*vtol.Jxz);
vtol.Gamma1   = vtol.Jxz*(vtol.Jx-vtol.Jy+vtol.Jz)/vtol.Gamma;
vtol.Gamma2   = (vtol.Jz*(vtol.Jz-vtol.Jy)+vtol.Jxz*vtol.Jxz)/vtol.Gamma;
vtol.Gamma3   = vtol.Jz/vtol.Gamma;
vtol.Gamma4   = vtol.Jxz/vtol.Gamma;
vtol.Gamma5   = (vtol.Jz-vtol.Jx)/vtol.Jy;
vtol.Gamma6   = vtol.Jxz/vtol.Jy;
vtol.Gamma7   = ((vtol.Jx-vtol.Jy)*vtol.Jx+vtol.Jxz*vtol.Jxz)/vtol.Gamma;
vtol.Gamma8   = vtol.Jx/vtol.Gamma;

vtol.C_L_0         = 0.23;
vtol.C_D_0         = 0.0434;
vtol.C_m_0         = 0.135;

vtol.C_L_alpha     = 5.6106;
vtol.C_D_alpha     = 0.3;
vtol.C_m_alpha     = -2.7397;

vtol.C_L_q         = 7.9543;
vtol.C_D_q         = 0.0;
vtol.C_m_q         = -38.2067;
vtol.C_L_delta_e   = 0.13;
vtol.C_D_delta_e   = 0.0135;

vtol.M             = 50;
vtol.alpha0        = 0.4712;
vtol.epsilon       = 0.1592;
vtol.C_D_p         = 0.0437;

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

vtol.C_ell_delta_a = -0.1695;
vtol.C_n_delta_r   = -0.0693;
vtol.C_m_delta_e   = -0.9918;

vtol.C_n_delta_a   = 0.0108;
vtol.C_Y_delta_r   = 0.1914;
vtol.C_ell_delta_r = 0.0024;

vtol.C_prop        = 1.0;

vtol.k_motor       = 60;
vtol.k_T_P         = 0;
vtol.k_Omega       = 0;

% wind parameters
vtol.wind_n = 0;
vtol.wind_e = 0;
vtol.wind_d = 0;
vtol.L_u = 200;
vtol.L_v = 200;
vtol.L_w = 50;
vtol.sigma_u = 1.06; 
vtol.sigma_v = 1.06;
vtol.sigma_w = .7;


% run trim commands
[x_trim,u_trim,y_trim,dx_trim]=compute_trim('mavsim_trim',vtol.Va0,gamma,R);
vtol.u_trim = u_trim;
vtol.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
vtol.pn0    = 0;  % initial North position
vtol.pe0    = 0;  % initial East position
vtol.pd0    = 0;  % initial Down position (negative altitude)
vtol.u0     = x_trim(4);  % initial velocity along body x-axis
vtol.v0     = x_trim(5);  % initial velocity along body y-axis
vtol.w0     = x_trim(6);  % initial velocity along body z-axis
vtol.phi0   = x_trim(7);  % initial roll angle
vtol.theta0 = x_trim(8);  % initial pitch angle
vtol.psi0   = x_trim(9);  % initial yaw angle
vtol.p0     = x_trim(10);  % initial body frame roll rate
vtol.q0     = x_trim(11);  % initial body frame pitch rate
vtol.r0     = x_trim(12);  % initial body frame yaw rate

%-------------------Used In AutoPilot------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
  %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Trim Constants~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vtol.Va_trim      = sqrt(x_trim(4)*x_trim(4)+x_trim(5)*x_trim(5)+x_trim(6)*x_trim(6));
    vtol.alpha_trim   = atan(x_trim(6)/x_trim(4));%0;
    vtol.beta_trim    = asin(x_trim(5)/vtol.Va_trim);%0;
    theta_trim     = x_trim(8);
    delta_e_trim   = u_trim(1);
    delta_t_trim   = u_trim(4); 
%------------------- Roll Angle-----------------------%
vtol.CPp        = vtol.Gamma3*vtol.C_ell_p+vtol.Gamma4*vtol.C_n_p;
vtol.CP_delta_a = vtol.Gamma3*vtol.C_ell_delta_a+vtol.Gamma4*vtol.C_n_delta_a;
vtol.a_phi1     = -0.5*vtol.rho*vtol.Va_trim*vtol.Va_trim*vtol.S_wing*vtol.b*vtol.CPp*vtol.b/2/vtol.Va_trim;
vtol.a_phi2     = 0.5*vtol.rho*vtol.Va_trim*vtol.Va_trim*vtol.S_wing*vtol.b*vtol.CP_delta_a;
% 
% %-----------------Course and Heading------------------%
% vtol.a_beta1    = -vtol.rho*vtol.Va0*vtol.S_wing*vtol.C_Y_beta/2/vtol.mass;
% vtol.a_beta2    = vtol.rho*vtol.Va0*vtol.S_wing*vtol.C_Y_beta/2/vtol.mass;
%-----------------Pitch Angle-------------------------%
vtol.a_theta1    = -vtol.rho*vtol.Va_trim*vtol.Va_trim*vtol.c*vtol.S_wing*vtol.C_m_q*vtol.c/4/vtol.Jy/vtol.Va_trim;
vtol.a_theta2    = -vtol.rho*vtol.Va_trim*vtol.Va_trim*vtol.c*vtol.S_wing*vtol.C_m_alpha/2/vtol.Jy;
vtol.a_theta3    =  vtol.rho*vtol.Va_trim*vtol.Va_trim*vtol.c*vtol.S_wing*vtol.C_m_delta_e/2/vtol.Jy;

%---------------------Airspeed------------------------%
vtol.a_V1        = (vtol.rho*vtol.Va_trim*vtol.S_wing*(vtol.C_D_0+vtol.C_D_alpha*vtol.alpha_trim+vtol.C_D_delta_e*delta_e_trim)+vtol.rho*vtol.S_prop*vtol.C_prop*vtol.Va_trim)/vtol.mass;
vtol.a_V2        = vtol.rho*vtol.S_prop*vtol.C_prop*vtol.k_motor*vtol.k_motor*delta_t_trim/vtol.mass;
vtol.a_V3        = vtol.gravity*cos(theta_trim-vtol.alpha_trim);

%---------------------Sideslip------------------------%
vtol.a_beta1     = -vtol.rho*vtol.Va_trim*vtol.S_wing*vtol.C_Y_beta/2/vtol.mass;
vtol.a_beta2     = vtol.rho*vtol.Va_trim*vtol.S_wing*vtol.C_Y_delta_r/2/vtol.mass;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%roll
vtol.delta_a_max=20*pi/180;
vtol.e_phi_max=15*pi/180;

vtol.zeta_phi=0.65;%0.7*5;


vtol.omega_n_phi = sqrt(abs(vtol.a_phi2)*vtol.delta_a_max*sqrt(1-vtol.zeta_phi^2)/vtol.e_phi_max);
    
    % set control gains based on zeta and wn
    vtol.kp_phi = vtol.omega_n_phi^2/vtol.a_phi2;
    vtol.kd_phi = (2*vtol.zeta_phi*vtol.omega_n_phi - vtol.a_phi1)/vtol.a_phi2;
    vtol.ki_phi = 0.1;
    
%Beta
vtol.delta_r_max=20*pi/180;
vtol.e_beta_max=15*pi/180;

vtol.zeta_beta=.707;
vtol.beta_kp=(vtol.delta_r_max/vtol.e_beta_max)*sign(vtol.a_beta2);
vtol.omega_n_beta=(vtol.a_beta2*vtol.beta_kp+vtol.a_beta1)/2/vtol.zeta_beta;
vtol.beta_ki=vtol.omega_n_beta^2/vtol.a_beta2;

%course
vtol.W_x=15; %bandwidth separation for roll and course
% vtol.W_x=15; %bandwidth separation for roll and course
vtol.omega_n_chi=vtol.omega_n_phi/vtol.W_x;
vtol.zeta_chi=1.2;%.707;
vtol.kp_chi=0.5*vtol.zeta_chi*vtol.omega_n_chi*vtol.Va0/vtol.gravity;
vtol.ki_chi=vtol.omega_n_chi^2*vtol.Va0/vtol.gravity;
vtol.kd_chi=0.5;

%pitch
vtol.delta_e_max=20*pi/180;
vtol.e_theta_max=10*pi/180;
vtol.kp_theta=2*(vtol.delta_e_max/vtol.e_theta_max)*sign(vtol.a_theta3);
vtol.omega_n_theta=sqrt(vtol.a_theta2+(vtol.delta_e_max/vtol.e_theta_max)*abs(vtol.a_theta3));
vtol.zeta_theta=0.707;
vtol.kd_theta=(2*vtol.zeta_theta*vtol.omega_n_theta-vtol.a_theta1)/vtol.a_theta3;
vtol.K_theta_DC=(vtol.kp_theta*vtol.a_theta3)/(vtol.a_theta2+vtol.kp_theta*vtol.a_theta3);
vtol.ki_theta=0.1;

% altitude
vtol.W_h=12; %bandwidth separation pitch and altitude
vtol.omega_n_h=vtol.omega_n_theta/vtol.W_h;
vtol.zeta_h=0.707;
vtol.ki_h=vtol.omega_n_h^2/(vtol.K_theta_DC*vtol.Va0);
vtol.kp_h=(2*vtol.zeta_h*vtol.omega_n_h)/(vtol.K_theta_DC*vtol.Va0);
vtol.kd_h=0;

% airspeed with Pitch
vtol.W_V2=15; % bandwidth separation
%vtol.W_V2=16; % bandwidth separation
vtol.omega_n_V2=vtol.omega_n_theta/vtol.W_V2;
vtol.zeta_V2=.707;
%vtol.zeta_V2=.707*8;

vtol.ki_V2=-vtol.omega_n_V2^2/(vtol.K_theta_DC*vtol.gravity);
vtol.kp_V2=(vtol.a_V1-2*vtol.zeta_V2*vtol.omega_n_V2)/(vtol.K_theta_DC*vtol.gravity);
vtol.kd_V2=0.1;

% airspeed with throttle
vtol.omega_n_V=vtol.omega_n_V2;
vtol.zeta_V=.707*2;
vtol.ki_V=vtol.omega_n_V^2/vtol.a_V2;
vtol.kp_V=5*(2*vtol.zeta_V*vtol.omega_n_V-vtol.a_V1)/vtol.a_V2;
vtol.kd_V=0.1;



vtol.altitude_take_off_zone=20;
vtol.altitude_hold_zone=30;
vtol.climb_pitch=20*pi/180;
vtol.theta_max=30*pi/180;
vtol.phi_max=30*pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% number of waypoints in data structure
vtol.size_waypoint_array = 100;
vtol.R_min = vtol.Va0^2/vtol.gravity/tan(vtol.phi_max);

% create random city map
city_width      = 2000;  % city size 
building_height = 300;   % maximum height
num_blocks      = 5;    % number of blocks in city
street_width    = 1;   % percent of block that is street.

vtol.map = createWorld(city_width, building_height, num_blocks, street_width);

