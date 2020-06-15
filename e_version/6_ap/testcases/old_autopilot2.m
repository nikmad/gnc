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
chi_c    = uu(3+NN);  % commanded course (rad)
    
%     % If phi_c_ff is specified in Simulink model, then do the following
%     phi_c_ff = uu(4+NN);  % feedforward roll command (rad)
%     NN = NN+4;
    
% If no phi_c_ff is included in inputs in Simulink model, then do this
NN = NN+3;
phi_c_ff = 0;

t = uu(1+NN);   % time


flag = 2;
if t==0
    flag = 1; % for initialization, flag is 1. For rest all cases it is 2;
end

%----------------------------------------------------------
% lateral autopilot
%chi_ref = wrap(chi_c, chi);
if t==0
    phi_c   = 0.0;
    delta_r = 0.0;
else
    %phi_c = 23
    phi_c   = course_with_roll(chi_c, chi, flag, AP)
    delta_r = yaw_damper(r, flag, AP)
end

delta_a = roll_with_aileron(phi_c, phi, p, flag, AP)

%----------------------------------------------------------
% longitudinal autopilot

%h_ref = sat1(h_c, h+AP.altitude_zone, h-AP.altitude_zone);
h_takeoff = 1000;
h_hold = 4000;
theta_takeoff = 10*pi/180;

if h < h_takeoff
    delta_t = 1
    theta_c = theta_takeoff
elseif (h >= h_takeoff && h < h_c - h_hold)
    delta_t = 1
    theta_c = airspeed_with_pitch(Va_c, Va, flag, AP)
elseif (h >= h_c-h_hold && h < h_c + h_hold)
    delta_t = airspeed_with_throttle(Va_c, Va, flag, AP, vtol)
    theta_c = altitude_with_pitch(h_c, h, flag, AP)
elseif h >= h_c + h_hold
    delta_t = 0
    theta_c = airspeed_with_pitch(Va_c, Va, flag, AP)
end

delta_e = pitch_with_elevator(theta_c, theta, q, flag, AP)

% limit range of throttle setting to [0,1]
delta_t = sat1(delta_t, 1, 0)

%----------------------------------------------------------
% create outputs

delta = [delta_e; delta_a; delta_r; delta_t]
size(delta)
% commanded (desired) states
x_command = [0; 0; h_c; Va_c; 0; 0; phi_c; theta_c; chi_c; 0; 0; 0]
size(x_command)

y = [delta; x_command];
 
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% course_with_roll
%   - regulate heading using the roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c_sat = course_with_roll(chi_c, chi, flag, AP)

% persistent integrator;
% persistent differentiator;
% persistent error_d1;

if flag==1 % reset (initialize) persistent variables
           % when flag==1
    integrator = 0;
    differentiator = 0;
    error_d1 = 0; % _d1 means delayed by one time step
end

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = chi_c;
y   = chi;

kp  = AP.course_kp;
ki  = AP.course_ki;
kd  = 0;

limit = 15*pi/180;

[phi_c_sat, integrator, differentiator, error_d1] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, integrator, differentiator, error_d1);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_with_aileron
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_with_aileron(phi_c, phi, p, flag, AP)

% persistent integrator;
% persistent error_d1;

if flag==1 % reset (initialize) persistent variables
           % when flag==1
    integrator = 0;
    error_d1 = 0; % _d1 means delayed by one time step
end

Ts  = AP.Ts; % sampling time

y_c = phi_c;
y   = phi;
ydot = p;

kp  = AP.roll_kp;
ki  = 0; % in ppt it was mentioned not to use integrator on Roll loop though it is given in the textbook
kd  = AP.roll_kd;

limit = 45*pi/180;

[delta_a, integrator, error_d1] = pidloop_rate(y_c, y, ydot, kp, ki, kd, limit, Ts, integrator, error_d1);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_with_elevator
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_with_elevator(theta_c, theta, q, flag, AP)

% persistent integrator;
% persistent error_d1;

if flag==1 % reset (initialize) persistent variables
           % when flag==1
    integrator = 0;
    error_d1 = 0; % _d1 means delayed by one time step
end

Ts  = AP.Ts; % sampling time

y_c = theta_c;
y   = theta;
ydot = q;

kp  = AP.pitch_kp;
ki  = 0; 
kd  = AP.pitch_kd;

limit = 45*pi/180;

[delta_e, integrator, error_d1] = pidloop_rate(y_c, y, ydot, kp, ki, kd, limit, Ts, integrator, error_d1);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t_sat = airspeed_with_throttle(Va_c, Va, flag, AP, vtol)

% persistent integrator;
% persistent differentiator;
% persistent error_d1;

if flag==1 % reset (initialize) persistent variables
           % when flag==1
    integrator = 0;
    differentiator = 0;
    error_d1 = 0; % _d1 means delayed by one time step
end

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = Va_c;
y   = Va;

kp  = AP.kp_V;
ki  = AP.ki_V; 
kd  = 0;

limit = 1;

[delta_t_temp, integrator, differentiator, error_d1] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, integrator, differentiator, error_d1);

delta_t_sat = vtol.u_trim(4) + delta_t_temp;
 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c_sat = airspeed_with_pitch(Va_c, Va, flag, AP)

% persistent integrator;
% persistent differentiator;
% persistent error_d1;

if flag==1 % reset (initialize) persistent variables
           % when flag==1
    integrator = 0;
    differentiator = 0;
    error_d1 = 0; % _d1 means delayed by one time step
end

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = Va_c;
y   = Va;

kp  = AP.kp_V2;
ki  = AP.ki_V2; 
kd  = 0;

limit = 2*pi;

[theta_c_sat, integrator, differentiator, error_d1] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, integrator, differentiator, error_d1);
 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_with_pitch
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c_sat = altitude_with_pitch(h_c, h, flag, AP)

% persistent integrator;
% persistent differentiator;
% persistent error_d1;

if flag==1 % reset (initialize) persistent variables
           % when flag==1
    integrator = 0;
    differentiator = 0;
    error_d1 = 0; % _d1 means delayed by one time step
end

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = h_c;
y   = h;

kp  = AP.altitude_kp;
ki  = AP.altitude_ki; 
kd  = 0;

limit = 2*pi;

[theta_c_sat, integrator, differentiator, error_d1] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, integrator, differentiator, error_d1);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinated_turn_hold
%   - sideslip with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_r = sideslip_with_rudder(beta, flag, AP)

% persistent integrator;
% persistent differentiator;
% persistent error_d1;

if flag==1 % reset (initialize) persistent variables
           % when flag==1
    integrator = 0;
    differentiator = 0;
    error_d1 = 0; % _d1 means delayed by one time step
end

tau = AP.tau; % time constant 
Ts  = AP.Ts; % sampling time

y_c = 0;
y   = beta;

kp  = AP.sideslip_kp;
ki  = AP.sideslip_ki; 
kd  = 0;

limit = 45*pi/180;

[delta_r, integrator, differentiator, error_d1] = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, integrator, differentiator, error_d1);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% yaw_damper
%   - yaw rate with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_r = yaw_damper(r, flag, AP)

persistent xi;
persistent Ts;
persistent Kr;
persistent p_wo;

if flag==1 % reset (initialize) persistent variables
           % when flag==1
    xi = 0;
    Ts = AP.Ts;
    Kr = AP.yaw_damper_kp;
    p_wo = AP.p_wo;
end

xi = xi + Ts*(-p_wo*xi + Kr*r);
delta_r = -p_wo*xi + Kr*r;
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


function [u, integrator, differentiator, error_d1]  = pidloop(y_c, y, kp, ki, kd, limit, Ts, tau, integrator, differentiator, error_d1)

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

function [u, integrator, error_d1] = pidloop_rate(y_c, y, ydot, kp, ki, kd, limit, Ts, integrator, error_d1)

error = y_c - y; % compute the current error

integrator = integrator + (Ts/2)*(error + error_d1);

% update differentiator
 error_d1 = error; % update the error for next time through the loop
 
 u = sat(... % implement PID control
 kp * error +... % proportional term
 ki * integrator +... % integral term
 - kd * ydot,... 
 limit... % ensure abs(u)<=limit
 );
 
% implement integrator anti-windup
 if ki ~= 0
     u_unsat = kp*error + ki*integrator - kd*ydot;
     integrator = integrator + (1/ki) * (u - u_unsat);
 end

end






















 