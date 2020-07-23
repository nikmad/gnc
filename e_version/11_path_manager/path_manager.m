function out = path_manager(in,atp)

  persistent start_of_simulation
  
  t = in(end);
  if t==0
      start_of_simulation = 1;
  end

  NN = 0;
  num_waypoints = in(1+NN);
  if num_waypoints==0 % start of simulation
      flag   = 1;  % following straight line path
      Va_d   = atp.Va0; % desired airspeed along waypoint path
%       NN     = NN + 1 + 5*atp.size_waypoint_array;
      NN = 1+NN;
      pn        = in(1+NN);
      pe        = in(2+NN);
      h         = in(3+NN);
      chi       = in(9+NN);
      r         = [pn; pe; -h];
      q         = [cos(chi); sin(chi); 0];
      c         = [0; 0; 0];
      rho       = 0;
      lambda    = 0;
      state     =  in(1+NN:16+NN);
      flag_need_new_waypoints = 1;
      out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints]; % 33
  else
    waypoints = reshape(in(2+NN:5*atp.size_waypoint_array+1+NN),5,atp.size_waypoint_array);
  
%     if 1
    if abs(waypoints(4,1))>=2*pi
        % follows straight-lines and switches at waypoints
        out = path_manager_fillet(in,atp,start_of_simulation);   %33
        % smooths through waypoints with fillets
        %out = path_manager_fillet(in,atp,start_of_simulation);  
        start_of_simulation=0;
    else
        % follows Dubins paths between waypoint configurations
        out = path_manager_dubins(in,atp,start_of_simulation); 
        start_of_simulation=0;
    end
  end
  
end