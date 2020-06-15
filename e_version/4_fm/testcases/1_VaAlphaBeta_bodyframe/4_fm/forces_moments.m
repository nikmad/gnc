% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North    
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
%     u_wg_i  = wind(4); % gust along North
%     v_wg_i  = wind(5); % gust along East    
%     w_wg_i  = wind(6); % gust along Down 

    vtol = P;
    
    % Rotation matrix
    R_v_v1=[cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    R_v1_v2=[cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    R_v2_b=[1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    R_v_b=R_v2_b*R_v1_v2*R_v_v1;

    % Total wind vector in body-frame
    V_w = R_v_b * [w_ns; w_es; w_ds] + [u_wg; v_wg; w_wg]; % V_w = [u_w; v_w; w_w]

    % Body-frame components of airspeed vector
    u_r = u-V_w(1);
    v_r = v-V_w(2);
    w_r = w-V_w(3);

    %u_r = u-V_w(1);
    %v_r = v-V_w(2);
    %w_r = w-V_w(3);

    % compute air data
    Va = sqrt(u_r^2+v_r^2+w_r^2);
    alpha = atan2(w_r, u_r);
    beta = asin(v_r/Va);
    
%     Va = 0.001;
%     alpha = 0;
%     beta = 0;

    % Compute wind data in NED (inertial): this is done by simply multiplying 
    % Total wind vector in body-frame with rotation matrix that rotates from body to inertial frame
    V_v = R_v_b'*V_w;
    %w_n = V_v(1,1);
    %w_e = V_v(2,1);
    %w_d = V_v(3,1);

    w_n = V_v(1);
    w_e = V_v(2);
    w_d = V_v(3);
  
    mass = vtol.mass;
    g = vtol.gravity;
    rho = vtol.rho;
    S = vtol.S_wing;

    % compute external forces and torques on aircraft

    %====================================================
    % Gravity force
    %====================================================
    fg_x = -mass*g*sin(theta);
    fg_y = mass*g*cos(theta)*sin(phi);
    fg_z = mass*g*cos(theta)*cos(phi);

    %====================================================
    % Aerodynamic forces
    %====================================================
    C_X_alpha = -vtol.C_D_alpha*cos(alpha) + vtol.C_L_alpha*sin(alpha);
    C_Xq_alpha = -vtol.C_D_q*cos(alpha) + vtol.C_L_q*sin(alpha);
    C_X_delta_e_alpha = -vtol.C_D_delta_e*cos(alpha) + vtol.C_L_delta_e*sin(alpha);
    C_Z_alpha = -vtol.C_D_alpha*sin(alpha) - vtol.C_L_alpha*cos(alpha);
    C_Zq_alpha = -vtol.C_D_q*sin(alpha) - vtol.C_L_q*cos(alpha);
    C_Z_delta_e_alpha = -vtol.C_D_delta_e*sin(alpha) - vtol.C_L_delta_e*cos(alpha);

    fa_x = 0.5*rho*Va^2*S*(C_X_alpha + C_Xq_alpha*(vtol.c/(2*Va))*q + C_X_delta_e_alpha*delta_e);
    fa_y = 0.5*rho*Va^2*S*(vtol.C_Y_0 + vtol.C_Y_beta*beta + vtol.C_Y_p*(vtol.b/(2*Va))*p ...
           + vtol.C_Y_r*(vtol.b/(2*Va))*r + vtol.C_Y_delta_a*delta_a + vtol.C_Y_delta_r*delta_r);
    fa_z = 0.5*rho*Va^2*S*(C_Z_alpha + C_Zq_alpha*(vtol.c/(2*Va))*q + C_Z_delta_e_alpha*delta_e);
    
    %====================================================
    % Engines/Motors(propulsion system) forces
    %====================================================
    C_prop = 1.0; % nkm - this coeff not defined in vtol_parameters.m file but it was taken from Appendix-E
    k_motor = 80; % nkm - this coeff not defined in vtol_parameters.m file but it was taken from Appendix-E

    fp_x = 0.5*rho*vtol.S_prop*C_prop*((k_motor*delta_t)^2-Va^2);
    fp_y = 0.0;
    fp_z = 0.0;

    %====================================================
    % Total Force in Body frame
    %====================================================
    Force(1) =  fg_x+fa_x+fp_x; % fx
    Force(2) =  fg_y+fa_y+fp_y; % fy
    Force(3) =  fg_z+fa_z+fp_z; % fz
    
    %====================================================
    % Aerodynamic Moments
    %====================================================
    ell_a = 0.5*rho*Va^2*S*vtol.b*(vtol.C_ell_0 + vtol.C_ell_beta*beta ...
        + vtol.C_ell_p*(vtol.b/(2*Va))*p + vtol.C_ell_r*(vtol.b/(2*Va))*r ...
        + vtol.C_ell_delta_a*delta_a + vtol.C_ell_delta_r*delta_r);
    m_a = 0.5*rho*Va^2*S*vtol.c*(vtol.C_m_0 + vtol.C_m_alpha*alpha ...
        + vtol.C_m_q*(vtol.c/(2*Va))*q + vtol.C_m_delta_e*delta_e);
    n_a = 0.5*rho*Va^2*S*vtol.b*(vtol.C_n_0 + vtol.C_n_beta*beta ...
        + vtol.C_n_p*(vtol.b/(2*Va))*p + vtol.C_n_r*(vtol.b/(2*Va))*r ...
        + vtol.C_n_delta_a*delta_a + vtol.C_n_delta_r*delta_r);

    %====================================================
    % Engines/motors(propulsion system) Moments
    %====================================================
    k_T_p = 0; % nkm - this coeff not defined in vtol_parameters.m file but it was taken from Appendix-E
    k_omega = 0; % nkm - this coeff not defined in vtol_parameters.m file but it was taken from Appendix-E

    ell_p = -k_T_p*(k_omega*delta_t)^2;
    m_p = 0;
    n_p = 0;

    %====================================================
    % Total Moments in Body frame
    %====================================================

    Torque(1) = ell_a+ell_p;
    Torque(2) = m_a+m_p;   
    Torque(3) = n_a+n_p;

   out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
   
end



