function out = path_follow(in)
    
% For testing:
% >> pin = [2; 35; 10; 10; 10; 20; 10; 0; 100; 0; 0; 10; 1; 5; 5; 5; 30; 0.5; 0.1; 0.2; 0.2; 0.2; 0.1; 1; 1; 0.4; 20; 3; 2; 0.4]
% >> path_follow(pin)
    
    % path following gains
    chi_inf = 60*pi/180;  % approach angle for large distance from straight-line path
    k_path    = .01;  %0.1      % proportional gain for path following
    k_orbit   = 2.5; %.7       % proportional gain for orbit following
    gravity   = 9.81;

    NN = 0;
    
    % PATH input to path_follow
    flag      = in(1+NN);
    Va_d      = in(2+NN);
    r_path    = [in(3+NN); in(4+NN); in(5+NN)];
    q_path    = [in(6+NN); in(7+NN); in(8+NN)];
    c_orbit   = [in(9+NN); in(10+NN); in(11+NN)];
    rho_orbit = in(12+NN);
    lam_orbit = in(13+NN);
    
    NN = NN + 13;
    
    % STATE input to path_follow
    pn        = in(1+NN);
    pe        = in(2+NN);
    h         = in(3+NN);
    Va        = in(4+NN);
    alpha   = in(5+NN);
    beta    = in(6+NN);
    phi       = in(7+NN);
    theta     = in(8+NN);
    chi       = in(9+NN);
    p       = in(10+NN);
    q       = in(11+NN);
    r       = in(12+NN);
    Vg      = in(13+NN);
    wn      = in(14+NN);
    we      = in(15+NN);
    psi     = in(16+NN);
  
    flag_need_new_waypoints = in(17+NN);
    
%     NN = 19+NN; % states are taken from 'path' input rather than the 'states' input. Hence these 19 variables are not used
%     
%     t         = in(1+NN);
    
    t = in(end);
  
    switch flag
        case 1 % follow straight line path specified by r and q
          
            % compute wrapped version of path angle
            chi_q = atan2(q_path(2),q_path(1));
            while (chi_q - chi < -pi), chi_q = chi_q + 2*pi; end
            while (chi_q - chi > +pi), chi_q = chi_q - 2*pi; end

              n_lon = [-sin(chi_q); cos(chi_q); 0];

              prod=cross(q_path,[0;0;1]);
              unit_prod=prod/norm(prod);
              epi=[pn-r_path(1);pe-r_path(2);h-r_path(3)];
              epi_dot=dot(epi,unit_prod);
              s_i=epi-(epi_dot*unit_prod);

              s_n=s_i(1);
              s_e=s_i(2);
              s_d=s_i(3);

              h_c = -r_path(3)-sqrt(s_n^2+s_e^2)*(q_path(3)/sqrt(q_path(1)^2+q_path(2)^2));

             epy=n_lon'*([pn;pe;-h]-r_path);
             chi_c = chi_q-chi_inf*2*atan(k_path*epy)/pi; 
                  
          phi_ff =-0*pi/180;
           
        case 2 % follow orbit specified by c, rho, lam
        
          % commanded altitude is the height of the orbit  
          h_c = -c_orbit(3);
          % distance from orbit center
          d_s = sqrt((pn-c_orbit(1))^2+(pe-c_orbit(2))^2); 
     
          % the roll angle feedforward command
          phi_ff = atan2((pe-c_orbit(2)),(pn-c_orbit(1)));
          while (phi_ff-chi)<-pi
              phi_ff=phi_ff+2*pi;
          end
          while (phi_ff-chi)>pi
              phi_ff=phi_ff-2*pi;
          end
          
          % heading command
          chi_c = phi_ff+lam_orbit*((pi/2)+(atan((k_orbit*(d_s-rho_orbit)/rho_orbit))));
%           orbit_error = 
    end
  
    % command airspeed equal to desired airspeed
    Va_c = Va_d;
  
    % create output
    out = [Va_c; h_c; chi_c; phi_ff];
end


