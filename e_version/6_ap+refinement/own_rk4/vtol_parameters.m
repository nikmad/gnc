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
	vtol.pd0    = -100;  % initial Down position (negative altitude)
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
% 	vtol.mass = 2477.0;
	vtol.Jx   = 0.8244;
	vtol.Jy   = 1.135;
	vtol.Jz   = 1.759;
	vtol.Jxz  = 0.1204;
	vtol.S_wing        = 0.55;
	vtol.b             = 2.90;
	vtol.c             = 0.19;
% 	vtol.S_wing        = 21.55;
% 	vtol.b             = 21.90;
% 	vtol.c             = 2.19;
%     vtol.Jx   = 2000;
% 	vtol.Jy   = 2000;
% 	vtol.Jz   = 2500;
% 	vtol.Jxz  = 0;
    
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
vtol.C_m_delta_e   = -0.99;
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
vtol.C_ell_delta_a = 0.1;
vtol.C_n_delta_a   = -0.011;
vtol.C_Y_delta_r   = 0.19;
vtol.C_ell_delta_r = 0.0024;
vtol.C_n_delta_r   = -0.069;

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

% Va0 & Va_c = 35 m/s
% x_trim = [   -0.0000;    0.0000;    0.0000;   34.9999;    0.0000;    0.0817;   -0.0002;   -0.2595;    0.0000;   -0.0000;    0.0000;    0.0000];
% u_trim = [    0.0072;    0.0008;   -0.0001;    0.8504];
% y_trim = [   35.0000;    0.0023;    0.0000];
% dx_trim = [   33.8074;   0.0000;    9.0587;    0.0000;   -0.0000;    0.0000;   -0.0000;    0.0000;    0.0000;    0.0000;    0.0000;    0.0000];    
% 
% x_trim = [-5.15200557515842e-21;1.41693952619918e-20;2.40177480491380e-22;34.9999046340913;1.35585468084861e-29;0.0817043726631074;-0.000154114236564779;-0.259464975059149;1.62871097306918e-20;-7.77003935217688e-24;1.55679198644296e-23;1.81389850700417e-23];
% y_trim = [35.0000000000000;0.00233441276774055;3.87387051671033e-31];
% u_trim = [0.00717546365292010;0.000841503650544109;-0.000134152755883843;0.850374370499802];
% dx_trim = [33.8074039201151;1.25918069671345e-05;9.05866657858823;1.70433364391057e-12;-9.02062562483878e-17;5.12882666175348e-13;-1.25843730776463e-23;1.55707151553763e-23;1.87646888342659e-23;6.31724528171947e-13;6.20318210726493e-17;4.32402690118831e-14];

x_trim = [-2.21245825109628e-17;6.45873891158749e-18;9.93896624735786e-19;34.9997295101928;2.86565796419585e-05;0.137601644325444;0.000979413435600612;0.00393148459631273;4.56924175090322e-18;5.68279411608219e-22;-8.22241736553171e-23;2.40104883882192e-22];
y_trim = [35;0.00393148568003598;8.18759418341764e-07];
% u_trim = [0.00275528205727417;-0.00552149494630340;0.000881104548505455;1.07530405828199];
u_trim = [-0.002029833270454;0;0;1.075];
dx_trim = [34.9999999998391;-0.000106112311769187;1.13520304267922e-14;2.15863852045842e-10;7.87240728415303e-16;8.11310611771874e-13;5.69223068061707e-22;-8.24592961301541e-23;2.40026092262076e-22;7.97325039241678e-11;-3.10159105363246e-17;5.45752420523227e-12];


% Va0 & Va_c = 55 m/s
% x_trim = [1.54541061994259e-18;-1.05722584794413e-18;-2.45765398108037e-20;54.9824737941499;5.12250378679338e-25;-1.38837151930595;0.00138969708111913;-0.287045163780591;-4.21496100883842e-19;2.03591923405758e-21;-6.01641376668284e-21;1.27823205726357e-21];
% u_trim = [0.0835085782878534;-0.00304920575891813;0.000486105265914484;1.56600819526843];
% y_trim = [55;-0.0252458001842970;9.31364324871523e-27];
% dx_trim = [53.1259204108629;0.00192941522685515;14.2350474806379;1.69556241370421e-08;6.85555407008747e-15;4.89240937888178e-12;1.66105664704441e-21;-6.01818431184744e-21;1.32404356118770e-21;6.11479471199183e-09;2.45088991013569e-15;4.18545368991175e-10];

% Va0 and Va_c = 115 m/s
% x_trim = [-1.61522439784689e-15;2.00673266902554e-15;-5.00258410471707e-17;114.910028282053;9.86076131526265e-31;-4.54812051485846;0.00999018954041805;-0.301356627833600;-5.24763141403749e-16;-2.19739940201547e-24;-1.15151174434914e-24;-1.44850636275009e-24];
% u_trim = [0.123123417865208;-0.00499205854483912;0.000795835420191743;3.58537785913297];
% y_trim = [115.000000000000;-0.0395591911264802;8.57457505675013e-33];
% dx_trim = [111.081460730898;0.0454358302084968;29.7641901867899;1.11103045949978e-13;6.05577959669529e-17;1.42108545828818e-14;-1.74361672110097e-24;-1.13698366974534e-24;-1.52883512405095e-24;6.09491578941034e-14;2.33131533855055e-48;4.22337429146049e-15];

% Va0 and Va_c = 111 m/s
% x_trim = [1.82362853465809e-13;1.75129654972599e-14;3.77008189164533e-16;110.982862150693;4.84587416775287e-17;-1.95046375010802;2.37708140401950e-05;-0.279372037676639;-2.71682553604017e-15;-9.53823234829348e-23;-4.36497941152299e-22;-3.63182577449537e-22];
% u_trim = [0.0622717784624861;-7.37579394472645e-05;1.17585120857968e-05;5.35459408068322];
% y_trim = [111.000000000001;-0.0175726498824311;4.36565240338092e-19];
% dx_trim = [107.217766718078;4.63641108003493e-05;28.7289140063799;5.36674437182212e-10;-3.29888924934689e-16;-6.59447963011485e-13;8.80849570406331e-24;-4.36489307883466e-22;-3.77842358794451e-22;1.68226996777321e-11;-6.39624890928186e-15;-5.96525166832439e-17];
% 

    vtol.x_trim = x_trim;
    vtol.u_trim = u_trim;
    
    % set initial conditions to trim conditions
    % initial conditions
    vtol.pn0    = 0;           % initial North position
    vtol.pe0    = 0;           % initial East position
    vtol.pd0    = -100;   
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
     


     
     
