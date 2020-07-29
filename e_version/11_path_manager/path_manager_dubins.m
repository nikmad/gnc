function out = path_manager_dubins(in,atp,start_of_simulation)

  NN = 0;
  
  % Note that R is the actual radius of turn demanded where as 
  % atp.R_min is the physical limitation of the aircraft. R must be 
  % R >= atp.R_min. But for now by default I am setting it to atp.R_min.
  R = atp.R_min; 
  
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
  chi     = in(9+NN);
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
  
  p_s   = waypoints(1:3,ptr_a); 
  p_e   = waypoints(1:3,ptr_a+1);

  chi_s = waypoints(4,ptr_a);
  chi_e = waypoints(4,ptr_a+1);
  
  % Rotation matrix for rotation about z-axis by 90 deg.
  Rz1 = [0 -1 0; 1 0 0; 0 0 1];
  % Rotation matrix for rotation about z-axis by -90 deg.
  Rz2 = [0 1 0; -1 0 0; 0 0 1];
  
  if (norm(p_s-p_e) >= 3*R) && (R >= atp.R_min) 
      c_rs = p_s + R * Rz1 * [cos(chi_s); sin(chi_s); 0];
      c_ls = p_s + R * Rz2 * [cos(chi_s); sin(chi_s); 0];
      c_re = p_e + R * Rz1 * [cos(chi_e); sin(chi_e); 0];
      c_le = p_e + R * Rz2 * [cos(chi_e); sin(chi_e); 0];
  end

  nue = dot()/norm();




      

  
  
  out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints];

end





























