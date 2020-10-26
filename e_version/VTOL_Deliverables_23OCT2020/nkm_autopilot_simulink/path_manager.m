
function out = path_manager(in,P)
% function out = path_manager(in)

  persistent start_of_simulation
  
%   P.Va0 = 35;
%   P.size_waypoint_array = 100;
%   P.R_min = 35*35/9.81;
  
  t = in(end);
  if t==0,
      start_of_simulation = 1;
  end

  NN = 0;
  num_waypoints = in(1+NN);
  if num_waypoints==0, % start of simulation
      flag   = 1;  % following straight line path
      Va_d   = P.Va0; % desired airspeed along waypoint path
%       Va_d = 35;
      NN = NN + 1 + 5*P.size_waypoint_array;
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
      out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints];
  else
    waypoints = reshape(in(2+NN:5*P.size_waypoint_array+1+NN),5,P.size_waypoint_array);

%     if 1
%         flag =1;
%         Va_d = P.Va0;
%         NN = NN +1 + 5*P.size_waypoint_array;
%         pn      = in(1+NN);
%       pe        = in(2+NN);
%       h         = in(3+NN);
%       chi       = in(9+NN);
%       r         = [pn; pe; -h];
%       q         = [cos(chi); sin(chi); 0];
%       c         = [0; 0; 0];
%       rho       = 0;
%       lambda    = 0;
%       state     =  in(1+NN:16+NN);
%       flag_need_new_waypoints = 1;
% %       out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints];
%       
% %         out = path_manager_line(in,P,start_of_simulation);  % follows straight-lines and switches at waypoints
%         out = path_manager_fillet(in,P,start_of_simulation);  % smooths through waypoints with fillets
%         start_of_simulation=0;
%     else
    if 1
        out = path_manager_dubins(in,P,start_of_simulation); % follows Dubins paths between waypoint configurations
        start_of_simulation=0;
    end
  end

end