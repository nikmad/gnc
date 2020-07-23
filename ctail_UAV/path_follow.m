function out = path_follow(in,P)
  
  NN = 0;
  flag      = in(1+NN);
  Va_d      = in(2+NN);
  r_path    = [in(3+NN); in(4+NN); in(5+NN)];
  q_path    = [in(6+NN); in(7+NN); in(8+NN)];
  c_orbit   = [in(9+NN); in(10+NN); in(11+NN)];
  rho_orbit = in(12+NN);
  lam_orbit = in(13+NN);
  NN = NN + 13;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  chi       = in(9+NN);

  switch flag,
      case 1, % follow straight line path 
           k_path=0.05;
          chi_inf=40*pi/180;
          
          chi_q=atan2(q_path(2),q_path(1));
          while (chi_q-chi)<-pi
              chi_q=chi_q+2*pi;
          end
          while (chi_q-chi)>pi
              chi_q=chi_q-2*pi;
          end
          
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

      case 2, % follow orbit
           k_orbit=1.7; 

          h_c =-c_orbit(3) ;
          d_s=(sqrt((pn-c_orbit(1))^2+(pe-c_orbit(2))^2));
          phi_ff = atan2((pe-c_orbit(2)),(pn-c_orbit(1)));
          while (phi_ff-chi)<-pi
              phi_ff=phi_ff+2*pi;
          end
          while (phi_ff-chi)>pi
              phi_ff=phi_ff-2*pi;
          end
          chi_c =phi_ff+lam_orbit*((pi/2)+(atan((k_orbit*(d_s-rho_orbit)/rho_orbit))));
  end

  Va_c = Va_d;

  out = [Va_c; h_c; chi_c; phi_ff];
end


