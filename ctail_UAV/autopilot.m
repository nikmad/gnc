function y = autopilot(uu,P)


    % process inputs
    NN = 0;
   pn       = uu(1+NN);  % inertial North position
   pe       = uu(2+NN);  % inertial East position
   h        = uu(3+NN);  % altitude
   Va       = uu(4+NN);  % airspeed
   beta     = uu(6+NN);  % side slip angle
   phi      = uu(7+NN);  % roll angle
   theta    = uu(8+NN);  % pitch angle
   chi      = uu(9+NN);  % course angle
   p        = uu(10+NN); % body frame roll rate
   q        = uu(11+NN); % body frame pitch rate
   r        = uu(12+NN); % body frame yaw rate

    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    

    [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        
    y = [delta; x_command];
end
    
   
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
%     lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        phi_c   = course_hold(chi_c, chi, r, 1, P);
        delta_a = roll_hold(phi_c, phi, p, 1, P);
        delta_r=0;
    else
        
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_a = roll_hold(phi_c, phi, p, 0, P); 
        delta_r=0;
    end
%        delta_a = roll_hold(phi_c, phi, p, P);  
  
    
    %----------------------------------------------------------
    % longitudinal autopilot

    persistent altitude_state;
    persistent initialize_integrator;
    if t==0,
        if h<=P.altitude_take_off_zone,     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone, 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone, 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    else
        if h<=P.altitude_take_off_zone,     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone, 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone, 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
       
    end
    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            delta_t=1;
            theta_c=P.climb_pitch;
            
        case 2,  % climb zone
            delta_t=0.7;
            theta_c=airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
%             theta_c=P.climb_pitch;
             
        case 3, % descend zone
            delta_t=0;
            theta_c=airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);

        case 4, % altitude hold zone
            delta_t=airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
            theta_c=altitude_hold(h_c, h, initialize_integrator, P);
    end
    if t==0
        delta_e = pitch_hold(theta_c, theta, q, 1, P);
    else
        delta_e = pitch_hold(theta_c, theta, q, 0, P);
    end
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
  error = phi_c - phi;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.kp_phi * error;
  
  % integral term
  ui = P.ki_phi * integrator;
  
  % derivative term
  ud = -P.kd_phi*p;
  
  
  % implement PID control
  delta_a = sat(up + ui + ud, 20*pi/180, -20*pi/180);
  

end

function phi_c = course_hold(chi_c, chi, r, flag, P)
    persistent integratorchi
    persistent error_d1chi
    if flag==1
        integratorchi=0;
        error_d1chi=0;
    end
    error=chi_c-chi;
    integratorchi=integratorchi+(P.Ts/2)*(error+error_d1chi);
    error_d1chi=error;
    phi_c=sat(P.kp_chi*error+P.ki_chi*integratorchi,P.phi_max,-P.phi_max);
    phi_c_unsat=P.kp_chi*error+P.ki_chi*integratorchi;
    integratorchi=integratorchi+(P.Ts/P.ki_chi)*(phi_c-phi_c_unsat);
end

function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
    persistent integratorV2
    persistent error_d1V2
    if flag==1
        integratorV2=0;
        error_d1V2=0;
    end
    error=Va_c-Va;
    integratorV2=integratorV2+(P.Ts/2)*(error+error_d1V2);
    error_d1V2=error;
    theta_c=sat(P.kp_V2*error+P.ki_V2*integratorV2,P.theta_max,-P.theta_max);
    theta_c_unsat=P.kp_V2*error+P.ki_V2*integratorV2;
    integratorV2=integratorV2+(P.Ts/P.ki_V2)*(theta_c-theta_c_unsat);
end

function delta_e = pitch_hold(theta_c, theta, q, flag, P)
    persistent integratortheta
    persistent error_d1theta
    if flag==1
        integratortheta=0;
        error_d1theta=0;
    end
    error=theta_c-theta;
    differentiator=q;
    integratortheta=integratortheta+(P.Ts/2)*(error+error_d1theta);
    error_d1theta=error;
    delta_e=sat(P.kp_theta*error+P.ki_theta*integratortheta-P.kd_theta*differentiator,P.phi_max,-P.phi_max);
    delta_e_unsat=P.kp_theta*error+P.ki_theta*integratortheta-P.kd_theta*differentiator;
    integratortheta=integratortheta+(P.Ts/P.ki_theta)*(delta_e-delta_e_unsat); 

% 
%     differentiator=q;
%     kp=P.kp_theta;
%     kd=P.kd_theta;
%     delta_e=sat(kp*error-kd*differentiator,20*pi/180,-20*pi/180);
end

function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
    persistent integratorV
    persistent error_d1V
    if flag==1
        integratorV=0;
        error_d1V=0;
    end
    error=Va_c-Va;
    integratorV=integratorV+(P.Ts/2)*(error+error_d1V);
    error_d1V=error;
    delta_t=sat(P.kp_V*error+P.ki_V*integratorV,1,0);
    delta_t_unsat=P.kp_V*error+P.ki_V*integratorV;
    integratorV=integratorV+(P.Ts/P.ki_V)*(delta_t-delta_t_unsat);
end

function  theta_c = altitude_hold(h_c, h, flag, P)
    persistent integratorh
    persistent error_d1h
    if flag==1
        integratorh=0;
        error_d1h=0;
    end
    error=h_c-h;
    integratorh=integratorh+(P.Ts/2)*(error+error_d1h);
    error_d1h=error;
    theta_c=sat(P.kp_h*error+P.ki_h*integratorh,P.theta_max,-P.theta_max);
    theta_c_unsat=P.kp_h*error+P.ki_h*integratorh;
    integratorh=integratorh+(P.Ts/P.ki_h)*(theta_c-theta_c_unsat);
end

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 