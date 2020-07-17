function [AP] = compute_autopilot_gains(trans_func, A_lon, B_lon, A_lat, B_lat, vtol)
%load all the transfer function coeffs from the file
%tfile = matfile('trans_func.mat');
%tfun = tfile.trans_func;

% % AP stands for autopilot
AP.gravity = vtol.gravity;
% AP.sigma = this is the same as tau
AP.tau = 0.05; % time constant used in differentiator of PID
%AP.tau = 0.5;
AP.Va0 = vtol.Va_trim; 
AP.Ts  = vtol.Ts;

%--------------------------------
  %Design Parameters
%--------------------------------
    
    %ROLL
    AP.delta_a_max = 45*pi/180;
    AP.e_phi_max   = 15*pi/180; 
    AP.zeta_phi     = 3.85;
    %AP.zeta_phi     = 70;
    
    %COURSE (this is the outer loop in which there is the inner ROLL loop)
%     AP.zeta_course     = 11.5;
    AP.zeta_course     = 1.85;
%     AP.zeta_course     = 3.85;
    %AP.W_chi           = 15; %Bandwidth separation between inner ROLL loop and outer COURSE loop.
    
    %SIDESLIP loop
    AP.delta_r_max = 45*pi/180;
    AP.e_beta_max  = 15*pi/180;
    AP.zeta_sideslip = 0.8;
    
    %PITCH LOOP
    AP.delta_e_max = 45*pi/180;
    AP.e_theta_max = 10*pi/180;
    %AP.zeta_theta = 0.2;
    AP.zeta_theta = .60;
    %AP.omega_n_theta = 9.93;
    
    %ALTITUDE FROM PITCH LOOP
%     AP.W_h = 10;
%     AP.zeta_h = 20;
%     AP.W_h = 13;
%     AP.zeta_h = 1.8;
    AP.W_h = 43;
    AP.zeta_h = 1.8;
    
    %AIRSPEED FROM PITCH LOOP
    AP.W_V2 = 13;
    AP.zeta_V2 = 1.9;
    
    %AIRSPEED FROM THROTTLE
%     AP.zeta_V = 5.3;
%     AP.omega_n_V = 9.8;
    AP.zeta_V = 2;
    AP.omega_n_V = 0.8;

%----------roll loop-----------------   
% It was suggested in the PPT that rather than putting integrator in roll
% loop we will put it in the course loop
[num,den] = tfdata(trans_func(1));
AP.roll_kp = (AP.delta_a_max/AP.e_phi_max)*sign(num{1,1}(3));
AP.omega_n_phi = sqrt(abs(num{1,1}(3))*(AP.delta_a_max/AP.e_phi_max));
AP.roll_kd = (2*AP.zeta_phi*AP.omega_n_phi-den{1,1}(2))/(num{1,1}(3));

%----------sideslip loop-------------
[num,den] = tfdata(trans_func(8));
AP.sideslip_kp = (AP.delta_r_max / AP.e_beta_max)*sign(num{1,1}(2));
AP.sideslip_ki = (1/num{1,1}(2)) * ((den{1,1}(2) + num{1,1}(2)*AP.sideslip_kp)...
    /(2*AP.zeta_sideslip))^2;

u = vtol.x_trim(4);
v = vtol.x_trim(5);
w = vtol.x_trim(6);
phi = vtol.x_trim(7);
theta = vtol.x_trim(8);
p = vtol.x_trim(10);
r = vtol.x_trim(12);
g = AP.gravity;
rho = vtol.rho;
S = vtol.S_wing;
mass = vtol.mass;
b = vtol.b;

AP.d_beta = (1/AP.Va0)*(p*w - r*u + g*cos(theta)*sin(phi)) ...
            + (0.5*rho*AP.Va0*S/mass)*(vtol.C_Y_0 + vtol.C_Y_p*(b/(2*AP.Va0))*p ...
           + vtol.C_Y_r*(b/(2*AP.Va0))*r + vtol.C_Y_delta_a*vtol.u_trim(2));

%----------yaw damper-------------
Yv = A_lat(1,1);
Yr = A_lat(1,3);
Y_delta_r = B_lat(1,2);
Nv = A_lat(3,1);
Nr = A_lat(3,3);
N_delta_r = B_lat(3,2);

AP.omega_n_dr = sqrt(Yv*Nr - Yr*Nv);% dutch-roll frequency
AP.p_wo = AP.omega_n_dr/10; % pole of the washout filter
AP.yaw_damper_tau_r = (1/AP.p_wo);

value1 = (Nr*N_delta_r + Y_delta_r*Nv)/(N_delta_r)^2;

% AP.yaw_damper_kp below denotes Kr
AP.yaw_damper_kp = - value1 ...
                   + sqrt(value1^2 - (Yv^2 + Nr^2 + 2*Yr*Nv) / N_delta_r^2);

%----------course loop---------------
%AP.omega_n_chi = (1/AP.W_chi) * AP.omega_n_phi; 
% Errata suggests that - 
% Rather than keeping the bandwidth of the outer course loop at least a factor of ten
% below the bandwidth of the roll loop, better performance may result from
% setting the outer course-loop bandwidth at least a factor of ten slower than
% the frequency of the dutch-roll mode. That's why I'm using the following
% formula for frequency - 
%AP.omega_n_chi = AP.omega_n_dr/50;
AP.omega_n_chi = AP.omega_n_dr/30;
AP.course_kp = 2 * AP.zeta_course * AP.omega_n_chi * AP.Va0/AP.gravity;
AP.course_ki = (AP.omega_n_chi)^2 * AP.Va0/AP.gravity;
AP.d_chi = tan(vtol.x_trim(7)) - vtol.x_trim(7);

%----------pitch loop-------------
[num,den] = tfdata(trans_func(3));
AP.pitch_kp = (AP.delta_e_max/AP.e_theta_max)*sign(num{1,1}(3));
AP.omega_n_theta = sqrt(den{1,1}(3) + abs(num{1,1}(3))*(AP.delta_e_max/AP.e_theta_max));
AP.pitch_kd = (2*AP.zeta_theta*AP.omega_n_theta - den{1,1}(2))/(num{1,1}(3));
AP.K_theta_DC = AP.pitch_kp * num{1,1}(3)/(den{1,1}(3) + AP.pitch_kp * num{1,1}(3));

% Modified using errata
% AP.pitch_kp = (AP.omega_n_theta^2 - den{1,1}(3)) / num{1,1}(3);
% AP.pitch_kd = (2 * AP.zeta_theta * AP.omega_n_theta - den{1,1}(2)) / num{1,1}(3);
% AP.K_theta_DC = AP.pitch_kp * num{1,1}(3) / AP.omega_n_theta^2;

%----------altitude hold using commanded pitch-------------
AP.omega_n_h = (1/AP.W_h)*AP.omega_n_theta;
AP.altitude_ki = AP.omega_n_h^2 / (AP.K_theta_DC * AP.Va0);
AP.altitude_kp = (2 * AP.zeta_h * AP.omega_n_h)/(AP.K_theta_DC * AP.Va0);
AP.d_h = (u*sin(theta)-AP.Va0*theta) - v*sin(phi)*cos(theta) - w*cos(phi)*cos(theta);
%AP.altitude_zone = 

%----------airspeed hold using commanded pitch-------------
[num,den] = tfdata(trans_func(7));
AP.omega_n_V2 = (1/AP.W_V2) * AP.omega_n_theta;
AP.kp_V2 = (den{1,1}(2) - 2 * AP.zeta_V2 * AP.omega_n_V2)...
                        /(AP.K_theta_DC * AP.gravity);
AP.ki_V2 = - AP.omega_n_V2^2 / (AP.K_theta_DC * AP.gravity);

%---------airspeed hold using throttle---------------
[num,den] = tfdata(trans_func(6));
AP.ki_V = AP.omega_n_V^2 / num{1,1}(2);
AP.kp_V = (2 * AP.zeta_V * AP.omega_n_V - den{1,1}(2)) / num{1,1}(2);

end






















