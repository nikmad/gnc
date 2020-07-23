% Author: Nikhil Madduri
% Created: 20/Jul/2020
% This function converts the output of the waypoint code of velocity
% demand (Va_c), course demand (chi_c) and altitude demand (h_c) into
% V_d = [u_d, v_d, w_d] which are velocity components obtained after transforming
% the inertial frame ground velocity (Vg) into body frame.

function [V_d] = waypoint_convert(Va_c, h_c, chi_c, x, wind)

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

    w_ns    = wind(1); % steady wind - North    
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    Vg_b = norm([u v w]); % Vg which is in inertial frame when converted to body-frame, its components are u,v,w

   % Rotation matrix
    R_v_v1=[cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    R_v1_v2=[cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    R_v2_b=[1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    R_v_b=R_v2_b*R_v1_v2*R_v_v1;

    % Total wind vector in body-frame
    V_w = R_v_b * [w_ns; w_es; w_ds] + [u_wg; v_wg; w_wg]; % V_w = [u_w; v_w; w_w]
    % Total wind vector in inertial-frame
    V_ned = R_v_b' * V_w;
    wn = V_ned(1);
    we = V_ned(2);
    wd = V_ned(3);
    
    % Body-frame components of airspeed vector
    u_r = u-V_w(1);
    v_r = v-V_w(2);
    w_r = w-V_w(3);

    % compute air data
    Va = sqrt(u_r^2+v_r^2+w_r^2);
    alpha = atan2(w_r, u_r);
    beta = asin(v_r/Va);

    gamma_a = theta - alpha;
    
    gamma = asin()
    
end

