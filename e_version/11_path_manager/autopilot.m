function y = autopilot(uu, AP, vtol)
% autopilot for vtolsim
% process inputs
NN = 0;
pn       = uu(1+NN);  % inertial North position
pe       = uu(2+NN);  % inertial East position
h        = uu(3+NN);  % altitude
Va       = uu(4+NN);  % airspeed
alpha    = uu(5+NN);  % angle of attack
beta     = uu(6+NN);  % side slip angle
phi      = uu(7+NN);  % roll angle
theta    = uu(8+NN);  % pitch angle
chi      = uu(9+NN);  % course angle
p        = uu(10+NN); % body frame roll rate
q        = uu(11+NN); % body frame pitch rate
r        = uu(12+NN); % body frame yaw rate
Vg       = uu(13+NN); % ground speed
wn       = uu(14+NN); % wind North
we       = uu(15+NN); % wind East
psi      = uu(16+NN); % heading
bx       = uu(17+NN); % x-gyro bias
by       = uu(18+NN); % y-gyro bias
bz       = uu(19+NN); % z-gyro bias
NN = NN+19;
Va_c     = uu(1+NN);  % commanded airspeed (m/s)
h_c      = uu(2+NN);  % commanded altitude (m)
% theta_c  = uu(3+NN);
chi_c    = uu(3+NN);  % commanded course (rad)
phi_c    = uu(4+NN);
    
    % If phi_c_ff is specified in Simulink model, then do the following
    %phi_c_ff = uu(4+NN);  % feedforward roll command (rad)
    %NN = NN+5;
    
% If no phi_c_ff is included in inputs in Simulink model, then do this
% NN = NN+3;
% phi_c_ff = 0;

t = uu(5+NN);   % time

persistent intg_prev;
persistent diff_prev;
persistent err_prev;

persistent xi_prev;
persistent Ts_prev;
persistent Kr_prev;
persistent pWo_prev;

% throttle_max = 2.5;
% throttle_limit = 2.5;
throttle_max = 2.0;
throttle_limit = 2.5;

flag = 2;
if t==0
    flag = 1; % for initialization, flag is 1. For rest all cases it is 2;
    intg_prev = [0;0;0;0;0;0;0];
    diff_prev = [0;0;0;0;0;0;0];
    err_prev  = [0;0;0;0;0;0;0];
    
    xi_prev = 0;
    Ts_prev = AP.Ts;
    Kr_prev = AP.yaw_damper_kp;
    pWo_prev = AP.p_wo;
%     Kr_prev = .996;
%     pWo_prev = .73;
%     Kr_prev = 1.5996;
%     pWo_prev = 1.573;

%     Kr_prev = 0.1;
%     pWo_prev = .1;
end

%beta = 5*pi/180;
%----------------------------------------------------------
% lateral autopilot
%chi_ref = wrap(chi_c, chi);
if flag==1
    phi_c   = 0.0;
    delta_r = 0.0;
else
    %phi_c = 23
    [phi_c, intg, diff, err] = course_with_roll(chi_c, chi, AP, intg_prev(1), diff_prev(1), err_prev(1));
    intg_prev(1) = intg;
    diff_prev(1) = diff;
    err_prev(1) = err;
    [delta_r, xi, Ts, Kr, pWo] = yaw_damper(r, xi_prev, Ts_prev, Kr_prev, pWo_prev);
    xi_prev = xi;
    Ts_prev = Ts;
    Kr_prev = Kr;
    pWo_prev= pWo;
%     [delta_r, intg, diff, err] = sideslip_with_rudder(beta, AP, intg_prev(2), diff_prev(2), err_prev(2));
%     intg_prev(2) = intg;
%     diff_prev(2) = diff;
%     err_prev(2) = err;
        
end



%[delta_a, intg, err] = roll_with_aileron(phi_c, phi, p, AP, intg_prev(2), err_prev(2));
% phi_c    = uu(5+NN); % Use it when testing roll loop. Comment it whenever giving course input (Course loop is the actual case we will be using in final Autopilot version).
[delta_a] = roll_with_aileron(phi_c, phi, p, AP);
% intg_prev(2) = intg;
% err_prev(2)  = err;

%----------------------------------------------------------
% longitudinal autopilot

%h_ref = sat1(h_c, h+AP.altitude_zone, h-AP.altitude_zone);
h_takeoff = 15;
h_hold = 20;
theta_takeoff = 15*pi/180;

% h_minus = h_c - h_hold
% h_plus = h_c + h_hold

if h < h_takeoff
    delta_t = throttle_max;
    theta_c = theta_takeoff;
%     h1 = h
elseif (h >= h_takeoff && h < h_c - h_hold)
    delta_t = throttle_max;
    [theta_c, intg, diff, err] = airspeed_with_pitch(Va_c, Va, AP, intg_prev(3), diff_prev(3), err_prev(3));
    intg_prev(3) = intg;
    diff_prev(3) = diff;
    err_prev(3) = err;
%     h2= h
elseif (h >= h_c-h_hold && h < h_c + h_hold)
    [delta_t, intg, diff, err] = airspeed_with_throttle(Va_c, Va, AP, vtol,throttle_limit, intg_prev(4), diff_prev(4), err_prev(4));
    if delta_t < 0
        delta_t = 0.0;
    end
    intg_prev(4) = intg;
    diff_prev(4) = diff;
    err_prev(4) = err;  
    [theta_c, intg, diff, err] = altitude_with_pitch(h_c, h, AP, intg_prev(5), diff_prev(5), err_prev(5));
    intg_prev(5) = intg;
    diff_prev(5) = diff;
    err_prev(5) = err;
%     h3= h
elseif h >= h_c + h_hold
    delta_t = 0.25;
    [theta_c, intg, diff, err] = airspeed_with_pitch(Va_c, Va, AP, intg_prev(6), diff_prev(6), err_prev(6));
    intg_prev(6) = intg;
    diff_prev(6) = diff;
    err_prev(6) = err;
%     h4= h
end

%[delta_e, intg, err] = pitch_with_elevator(theta_c, theta, q, AP, intg_prev(7), err_prev(7));
%theta_c    = uu(3+NN); % Use it when testing pitch loop. Comment it in actual case we will be using in final Autopilot version
[delta_e] = pitch_with_elevator(theta_c, theta, q, AP);
%delta_e = delta_e;
% intg_prev(7) = intg;
% err_prev(7)  = err;

% limit range of throttle setting to [0,1]
%delta_t = sat1(delta_t, 1, 0);

%----------------------------------------------------------
% create outputs

delta = [delta_e; delta_a; delta_r; delta_t];
% commanded (desired) states
x_command = [0; 0; h_c; Va_c; 0; 0; phi_c; theta_c; chi_c; 0; 0; 0];

y = [delta; x_command];

y_prev = [delta; x_command];
 
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% course_with_roll
%   - regulate heading using the roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
function [phi_c_sat, intg_new, diff_new, err_new] = course_with_roll(chi_c, chi, AP, intg, diff, err)

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = chi_c;
y   = chi;

kp  = AP.course_kp;
ki  = AP.course_ki;
kd  = 0;

limit = 45*pi/180;

[phi_c_sat, intg_new, diff_new, err_new] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, intg, diff, err);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_with_aileron
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta_a] = roll_with_aileron(phi_c, phi, p, AP)

Ts  = AP.Ts; % sampling time

y_c = phi_c;
y   = phi;
ydot = p;

kp  = AP.roll_kp;
%ki  = 0; % in ppt it was mentioned not to use integrator on Roll loop though it is given in the textbook
kd  = AP.roll_kd;

limit = 30*pi/180;

%[delta_a, intg_new, err_new] = pidloop_rate(y_c, y, ydot, kp, ki, kd, limit, Ts, intg, err);
[delta_a] = pidloop_rate(y_c, y, ydot, kp, kd, limit);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_with_elevator
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function [delta_e, intg_new, err_new] = pitch_with_elevator(theta_c, theta, q, AP)
function [delta_e] = pitch_with_elevator(theta_c, theta, q, AP)

Ts  = AP.Ts; % sampling time

y_c = theta_c;
y   = theta;
ydot = q;

kp  = AP.pitch_kp;
%ki  = 0; 
kd  = AP.pitch_kd;

limit = 30*pi/180;

%[delta_e, intg_new, err_new] = pidloop_rate(y_c, y, ydot, kp, ki, kd, limit, Ts, intg, err);
[delta_e] = pidloop_rate(y_c, y, ydot, kp, kd, limit);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta_t_sat, intg_new, diff_new, err_new] = airspeed_with_throttle(Va_c, Va, AP, vtol, throttle_limit, intg, diff, err)

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = Va_c;
y   = Va;

kp  = AP.kp_V;
ki  = AP.ki_V; 
kd  = 0;

limit = throttle_limit;

[delta_t_sat, intg_new, diff_new, err_new] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, intg, diff, err);

delta_t_sat = vtol.u_trim(4) + delta_t_sat;
 
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [theta_c_sat, intg_new, diff_new, err_new] = airspeed_with_pitch(Va_c, Va, AP, intg, diff, err)

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = Va_c;
y   = Va;

kp  = AP.kp_V2;
ki  = AP.ki_V2; 
kd  = 0;

limit = 45*pi/180;

[theta_c_sat, intg_new, diff_new, err_new] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, intg, diff, err);
 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_with_pitch
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [theta_c_sat, intg_new, diff_new, err_new] = altitude_with_pitch(h_c, h, AP, intg, diff, err)

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = h_c;
y   = h;

kp  = AP.altitude_kp;
ki  = AP.altitude_ki; 
kd  = 0;

limit = 45*pi/180;

[theta_c_sat, intg_new, diff_new, err_new] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, intg, diff, err);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinated_turn_hold
%   - sideslip with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta_r, intg_new, diff_new, err_new] = sideslip_with_rudder(beta, AP, intg, diff, err)

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = 0;
y   = beta;

kp  = AP.sideslip_kp;
ki  = AP.sideslip_ki; 
kd  = 0;

limit = 20*pi/180;

[delta_r, intg_new, diff_new, err_new] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, intg, diff, err);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% yaw_damper
%   - yaw rate with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta_r, xi_new, Ts_new, Kr_new, pWo_new] = yaw_damper(r, xi, Ts, Kr, p_wo)

% persistent xi;
% persistent Ts;
% persistent Kr;
% persistent p_wo;

% if flag==1 % reset (initialize) persistent variables
%            % when flag==1
%     xi = 0;
%     Ts = AP.Ts;
%     Kr = AP.yaw_damper_kp;
%     p_wo = AP.p_wo;
% end

xi = xi + Ts*(-p_wo*xi + Kr*r);
delta_r = sat(-p_wo*xi + Kr*r, 20*pi/180);

xi_new = xi;
Ts_new = Ts;
Kr_new = Kr;
pWo_new = p_wo;

end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, limit)

    if in < -limit
        out = -limit;
    elseif in > limit
        out = limit;
    else
        out = in;
    end
end

function out = sat1(in, max, min)

    if in < min
        out = min;
    elseif in > max
        out = max;
    else
        out = in;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wrap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function chi_c = wrap(chi_c, chi)
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The PID algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             
function [u, integrator, differentiator, error_d1] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, intg, diff, err)

% if flag==1 % reset (initialize) persistent variables
%            % when flag==1
%     intg = 0;
%     diff = 0;
%     err = 0; % _d1 means delayed by one time step
% end
integrator = intg;
differentiator = diff;
error_d1 = err;

error = y_c - y; % compute the current error
 
integrator = integrator + (Ts/2)*(error + error_d1);

% update integrator
differentiator = (2*tau-Ts)/(2*tau+Ts)*differentiator...
 + 2/(2*tau+Ts)*(error - error_d1);

% update differentiator
 error_d1 = error; % update the error for next time through the loop
 
 u = sat(... % implement PID control
 kp * error +... % proportional term
 ki * integrator +... % integral term
 kd * differentiator,... % derivative term
 limit... % ensure abs(u)<=limit
 );
 
% implement integrator anti-windup
 if ki ~= 0
     u_unsat = kp*error + ki*integrator + kd*differentiator;
     integrator = integrator + (1/ki) * (u - u_unsat);
 end

end

%function [u, integrator, error_d1] = pidloop_rate(y_c, y, ydot, kp, ki, kd, limit, Ts, intg, err)
function [u] = pidloop_rate(y_c, y, ydot, kp, kd, limit)

% if flag==1 % reset (initialize) persistent variables
%            % when flag==1
%     integrator = 0;
%     error_d1 = 0; % _d1 means delayed by one time step
% end

%integrator = intg;
%error_d1 = err;

error = y_c - y; % compute the current error
 
%integrator = integrator + (Ts/2)*(error + error_d1);

% update differentiator
%error_d1 = error; % update the error for next time through the loop
 
 u = sat(... % implement PID control
 kp * error... % proportional term
 - kd * ydot,... 
 limit ... % ensure abs(u)<=limit
 );
 %ki * integrator +... % integral term
 
% implement integrator anti-windup
%  if ki ~= 0
%      %u_unsat = kp*error + ki*integrator - kd*ydot;
%      u_unsat = kp*error - kd*ydot;
%      %integrator = integrator + (1/ki) * (u - u_unsat);
%  end

end






















 