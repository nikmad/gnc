function out = path_manager(in,atp)

  persistent start_of_simulation
  
  t = in(end);
  
  if t==0
      start_of_simulation = 1;
  end

  NN = 0;
  
  % nkm - If you uncomment the following line and while running the simulink
  % model, if you intentionally crash the program by closing any of the
  % figure windows while the program is still running, a 'Diagnostic
  % Viewer' would pop up in which one can see the value of 'num_waypoints'
  % that this file is getting at each time step. You would observe that at
  % t=0, its value is not the actual num_waypoints you've defined in path 
  % planner (say you've defined 5 waypoints in path planner) but it is
  % shown as 0. Hence for t=0, num_waypoints = 0. For all other t>0,
  % num_waypoints = 5 (or whatever number of waypoints one has defined in 
  % the path planner). Hence whatever be the num_waypoints you've defined
  % in path planner, you would always enter the if loop 'if
  % num_waypoints==0' at t=0. However one has to check when implementing in
  % C code whether the num_waypoints==0 or the actual number (say 5) for
  % t=0. All this depends on the sequence of file execution in C.
  
  num_waypoints = in(1+NN); 
  
  if num_waypoints==0 % start of simulation
      flag   = 1;  % following straight line path
      Va_d   = atp.Va0; % desired airspeed along waypoint path
      NN     = NN + 1 + 5*atp.size_waypoint_array;
%       NN = 1+NN;
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
      out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints]; % 30
  else
      % nkm - the following line takes array and converts it into a matrix 
      % of 5 rows x num_waypoint columns that has all the waypoint
      % information concatenated into single array with 5 elements defining
      % each waypoint starting from 2nd element while the 1st element is
      % the number that tells the total number of waypoints input in path
      % planner.
      waypoints = reshape(in(2+NN:5*atp.size_waypoint_array+1+NN),5,atp.size_waypoint_array);
    
%   The following line checks if the course angle demand is greater than
%   2*pi.
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