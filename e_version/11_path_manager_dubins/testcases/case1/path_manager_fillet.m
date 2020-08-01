function out = path_manager_fillet(in,atp,start_of_simulation)

  NN = 0;
  num_waypoints = in(1+NN);
  waypoints = reshape(in(2+NN:5*atp.size_waypoint_array+1+NN),5,atp.size_waypoint_array);
  NN = NN + 1 + 5*atp.size_waypoint_array;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  % chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  state     =  in(1+NN:16+NN);
  NN = NN + 16;
  t         = in(1+NN);
 
  p = [pn; pe; -h];

  persistent waypoints_old   % stored copy of old waypoints
  persistent ptr_a           % waypoint pointer
  persistent ptr_b
  persistent state_transition % state of transition state machine
  persistent flag_need_new_waypoints % flag that request new waypoints from path planner
  
  if start_of_simulation || isempty(waypoints_old)
      waypoints_old = zeros(5,atp.size_waypoint_array);
      flag_need_new_waypoints = 0;
  end
  
  % if the waypoints have changed, update the waypoint pointer
  if min(min(waypoints==waypoints_old))==0
      ptr_a = 1;
      ptr_b = 2;
      waypoints_old = waypoints;
      state_transition = 1;
      flag_need_new_waypoints = 0;
  end
  
  % define current and next two waypoints
  
  w_a   = waypoints(1:3,ptr_a); 
  w_b    = waypoints(1:3,ptr_a+1);
  w_c   = waypoints(1:3,ptr_a+2);
  q_n    = (w_b-w_a)/norm(w_b-w_a);
  q_p    = (w_c-w_b)/norm(w_c-w_b);
  angle  = acos(-q_n'*q_p);

% define transition state machine
  switch state_transition
      case 1 % follow straight line from wpp_a to wpp_b
          flag   = 1;  % following straight line path
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          r      = w_a;
          q      = q_n;
%         q      = q/norm(q);
          c      = w_b;
          rho    = 0;
          lambda = 0;
          z      = w_b-((atp.R_min/tan(angle/2))*q_n);
          halfplane = (p(1:2)-z(1:2))'*q_n(1:2);
          if halfplane>=0
           state_transition=2;
           flag_need_new_waypoints = 0;
          end
          
      case 2 % follow orbit from wpp_a-wpp_b to wpp_b-wpp_c
          n_i    = (q_n-q_p)/norm(q_n-q_p);
          flag   = 2;  % following orbit
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          r      = w_a;
          q      = q_n;
          c      = w_b-((atp.R_min/sin(angle/2))*n_i);
          rho    = atp.R_min;
          lambda = sign(q_n(1)*q_p(2)-q_n(2)*q_p(1));
          z      = w_b+((atp.R_min/tan(angle/2))*q_p);
          halfplane = (p(1:2)-z(1:2))'*q_p(1:2);
          if halfplane>=0
              if (ptr_a<num_waypoints-1) && (ptr_a<ptr_b+1)
                  ptr_a=ptr_a+1;
              end
             flag_need_new_waypoints = 0;ptr_b=ptr_b+1;
             state_transition=1;% state=1;
          end
  end
  
  out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints];

end