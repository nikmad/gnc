function cplots()
%
%   M = readmatrix('nikstates.txt')
    M=importdata('nikstates.txt');
%   save('nikstates', 'M');
  
  figure(2);
  
  subplot(8,2,1)
  plot(M(:,1),M(:,2),'.-r');
  ylabel('Pn');
  
  subplot(8,2,2)
  plot(M(:,1),M(:,14),'.-r');
  ylabel('Va');
  
  subplot(8,2,3)
  plot(M(:,1),M(:,3),'.-r');
  ylabel('Pe');
  
  subplot(8,2,4)
  plot(M(:,1),(180/pi)*M(:,15),'.-r');
  ylabel('\alpha');
  
  subplot(8,2,5)
  plot(M(:,1),-M(:,4),'.-g');
   ylabel('h');
    
  subplot(8,2,6)
  plot(M(:,1),-(180/pi)*M(:,16),'.-g');
   ylabel('\beta');
  
  subplot(8,2,7)
  plot(M(:,1),(180/pi)*M(:,8),'.-g');
   ylabel('\phi');
    
  subplot(8,2,8)
  plot(M(:,1),(180/pi)*M(:,11),'.-g');
   ylabel('p');
  
  subplot(8,2,9)
  plot(M(:,1),(180/pi)*M(:,9),'.-r');
   ylabel('\theta');
    
  subplot(8,2,10)
  plot(M(:,1),(180/pi)*M(:,12),'.-r');
   ylabel('q');
    
  subplot(8,2,11)
  plot(M(:,1),(180/pi)*M(:,17),'.-r');
     ylabel('\chi');
    
  subplot(8,2,12)
  plot(M(:,1),(180/pi)*M(:,13),'.-r');
     ylabel('r');
    
  subplot(8,2,13)
  plot(M(:,1),(180/pi)*M(:,18),'.-r');  
     ylabel('\delta_e');
  
  subplot(8,2,14)
  plot(M(:,1),(180/pi)*M(:,19),'.-r');
       ylabel('\delta_a');
    
  subplot(8,2,15)
  plot(M(:,1),(180/pi)*M(:,20),'.-r');
       ylabel('\delta_r');
  
  subplot(8,2,16)
  plot(M(:,1),M(:,21),'.-r');
       ylabel('\delta_t');

%  for i=1:length(M)
%     % process inputs to function
%     pn          = M(i,2);             % North position (meters)
%     pe          = M(i,3);             % East position (meters)
%     h           = M(i,4);            % altitude (meters)
%     u           = M(i,5);             % body velocity along x-axis (meters/s)
%     v           = M(i,6);             % body velocity along y-axis (meters/s)
%     w           = M(i,7);             % body velocity along z-axis (meters/s)
%     phi         = M(i,8);      % roll angle (degrees)   
%     theta       = M(i,9);      % pitch angle (degrees)
%     psi         = M(i,10);             % yaw angle (radians)
%     p           = M(i,11);     % body angular rate along x-axis (degrees/s)
%     q           = M(i,12);     % body angular rate along y-axis (degrees/s)
%     r           = M(i,13);     % body angular rate along z-axis (degrees/s)
%     Va          = 0;            % airspeed (m/s)
%     alpha       = 0;     % angle of attack (degrees)
%     beta        = 0;     % side slip angle (degrees)
%     wn          = 0;            % wind in the North direction
%     we          = 0;            % wind in the East direction
%     wd          = 0;            % wind in the Down direction
%     pn_c        = 0;            % commanded North position (meters)
%     pe_c        = 0;            % commanded East position (meters)
%     h_c         = 0;            % commanded altitude (meters)
%     Va_c        = 0;            % commanded airspeed (meters/s)
%     alpha_c     = 0;     % commanded angle of attack (degrees)
%     beta_c      = 0;     % commanded side slip angle (degrees)
%     phi_c       = 0;     % commanded roll angle (degrees)   
%     theta_c     = 0;     % commanded pitch angle (degrees)
%     chi_c       = 0;     % commanded course (degrees)
%     p_c         = 0;     % commanded body angular rate along x-axis (degrees/s)
%     q_c         = 0;     % commanded body angular rate along y-axis (degrees/s)
%     r_c         = 0;     % commanded body angular rate along z-axis (degrees/s)
%     pn_hat      = 0;            % estimated North position (meters)
%     pe_hat      = 0;            % estimated East position (meters)
%     h_hat       = 0;            % estimated altitude (meters)
%     Va_hat      = 0;            % estimated airspeed (meters/s)
%     alpha_hat   = 0;     % estimated angle of attack (degrees)
%     beta_hat    = 0;     % estimated side slip angle (degrees)
%     phi_hat     = 0;     % estimated roll angle (degrees)   
%     theta_hat   = 0;     % estimated pitch angle (degrees)
%     chi_hat     = 0;     % estimated course (degrees)
%     p_hat       = 0;     % estimated body angular rate along x-axis (degrees/s)
%     q_hat       = 0;     % estimated body angular rate along y-axis (degrees/s)
%     r_hat       = 0;     % estimated body angular rate along z-axis (degrees/s)
% %    Vg_hat      = uu(43);            % estimated groundspeed
% %    wn_hat      = uu(44);            % estimated North wind
% %    we_hat      = uu(45);            % estimated East wind
% %    psi_hat     = 180/pi*uu(46);     % estimated heading
% %    bx_hat      = uu(47);            % estimated x-gyro bias
% %    by_hat      = uu(48);            % estimated y-gyro bias
% %    bz_hat      = uu(49);            % estimated z-gyro bias
%     delta_e     = 0;     % elevator angle (degrees)
%     delta_a     = 0;     % aileron angle (degrees)
%     delta_r     = 0;     % rudder angle (degrees)
%     delta_t     = 0;            % throttle setting (unitless)
%     t           = M(i,1);            % simulation time
%     
%     % compute course angle
%     chi = 180/pi*atan2(Va*sin(psi)+we, Va*cos(psi)+wn);
%     %Va = 50;
% 
% %     gamma_a = theta-alpha;
% %     gamma_a_c = theta_c-alpha_c;
% %     gamma_a_hat = theta_hat-alpha_hat;
%     
%     % define persistent variables 
%     persistent pn_handle
%     persistent pe_handle
%     persistent h_handle
%     persistent Va_handle
%     %persistent gamma_a_handle
%     persistent alpha_handle
%     persistent beta_handle
%     persistent phi_handle
%     persistent theta_handle
%     persistent chi_handle
%     persistent p_handle
%     persistent q_handle
%     persistent r_handle
%     persistent delta_e_handle
%     persistent delta_a_handle
%     persistent delta_r_handle
%     persistent delta_t_handle
%     
% 
%   % first time function is called, initialize plot and persistent vars
%     %if t==0
%         figure(2), clf
% 
%         subplot(8,2,1)
%         hold on
%         grid on
%         %plot(M(:,1), M(i,:),'--go');
%         %pn_handle = graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', []);
%         
%         subplot(8,2,2)
%         hold on
%         grid on
%         Va_handle = graph_y_yhat_yd(t, Va, Va_hat, Va_c, 'V_a', []);
%         %gamma_a_handle = graph_y_yhat_yd(t, gamma_a, gamma_a_hat, gamma_a_c, '(\gamma)_a', []);
% 
%         subplot(8,2,3)
%         hold on
%         grid on
%         pe_handle = graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', []);
% 
%         subplot(8,2,4)
%         hold on
%         grid on
%         alpha_handle = graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', []);
% 
%         subplot(8,2,5)
%         hold on
%         grid on
%         h_handle = graph_y_yhat_yd(t, h, h_hat, h_c, 'h', []);
% 
%         subplot(8,2,6)
%         hold on
%         grid on
%         beta_handle = graph_y_yhat_yd(t, beta, beta_hat, beta_c, '\beta', []);
% 
%         subplot(8,2,7)
%         hold on
%         grid on
%         phi_handle = graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi', []);
%         
%         subplot(8,2,8)
%         hold on
%         grid on
%         p_handle = graph_y_yhat_yd(t, p, p_hat, p_c, 'p', []);
%         
%         subplot(8,2,9)
%         hold on
%         grid on
%         theta_handle = graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', []);
%         
%         subplot(8,2,10)
%         hold on
%         grid on
%         q_handle = graph_y_yhat_yd(t, q, q_hat, q_c, 'q', []);
%         
%         subplot(8,2,11)
%         hold on
%         grid on
%         chi_handle = graph_y_yhat_yd(t, chi, chi_hat, chi_c, '\chi', []);
%         
%         subplot(8,2,12)
%         hold on
%         grid on
%         r_handle = graph_y_yhat_yd(t, r, r_hat, r_c, 'r', []);
%         
%         subplot(8,2,13)
%         hold on
%         grid on
%         delta_e_handle = graph_y(t, delta_e, [], 'b');
%         ylabel('\delta_e')
%         
%         subplot(8,2,14)
%         hold on
%         grid on
%         delta_a_handle = graph_y(t, delta_a, [], 'b');
%         ylabel('\delta_a')
% 
%         subplot(8,2,15)
%         hold on
%         grid on
%         delta_r_handle = graph_y(t, delta_r, [], 'b');
%         ylabel('\delta_r')
%         
%         subplot(8,2,16)
%         hold on
%         grid on
%         delta_t_handle = graph_y(t, delta_t, [], 'b');
%         ylabel('\delta_t')
%         
%     % at every other time step, redraw state variables
% %     else 
% %        graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', pn_handle);
% %        graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', pe_handle);
% %        graph_y_yhat_yd(t, h, h_hat, h_c, 'h', h_handle);
% %        graph_y_yhat_yd(t, Va, Va_hat, Va_c, 'V_a', Va_handle);
% %        %graph_y_yhat_yd(t, gamma_a, gamma_a_hat, gamma_a_c, '(\gamma)_a', gamma_a_handle);
% %        graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', alpha_handle);
% %        graph_y_yhat_yd(t, beta, beta_hat, beta_c, '\beta', beta_handle);
% %        graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi', phi_handle);
% %        graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', theta_handle);
% %        graph_y_yhat_yd(t, chi, chi_hat, chi_c, '\chi', chi_handle);
% %        graph_y_yhat_yd(t, p, p_hat, p_c, 'p', p_handle);
% %        graph_y_yhat_yd(t, q, q_hat, q_c, 'q', q_handle);
% %        graph_y_yhat_yd(t, r, r_hat, r_c, 'r', r_handle);
% %        graph_y(t, delta_e, delta_e_handle);
% %        graph_y(t, delta_a, delta_a_handle);
% %        graph_y(t, delta_r, delta_r_handle);
% %        graph_y(t, delta_t, delta_t_handle);
% %     end
%  end
% 
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % graph y with lable mylabel
% function handle = graph_y(t, y, handle, color)
%   
%   if isempty(handle)
%     handle    = plot(t,y,color);
%   else
%     set(handle,'Xdata',[get(handle,'Xdata'),t]);
%     set(handle,'Ydata',[get(handle,'Ydata'),y]);
%     %drawnow
%   end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % graph y and yd with lable mylabel
% function handle = graph_y_yd(t, y, yd, lab, handle)
%   
%   if isempty(handle)
%     handle(1)    = plot(t,y,'b');
%     handle(2)    = plot(t,yd,'g--');
%     ylabel(lab)
%     set(get(gca, 'YLabel'),'Rotation',0.0);
%   else
%     set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
%     set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
%     set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
%     set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
%     %drawnow
%   end
% 
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % plot the variable y in blue, its estimated value yhat in green, and its 
% % desired value yd in red, lab is the label on the graph
% function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
%   
%   if isempty(handle)
%     handle(1)   = plot(t,y,'b');
%     handle(2)   = plot(t,yhat,'g--');
%     handle(3)   = plot(t,yd,'r-.');
%     ylabel(lab)
%     set(get(gca,'YLabel'),'Rotation',0.0);
%   else
%     set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
%     set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
%     set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
%     set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
%     set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
%     set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
%     %drawnow
%   end
% 
% %
% %=============================================================================
% % sat
% % saturates the input between high and low
% %=============================================================================
% %
% function out=sat(in, low, high)
% 
%   if in < low
%       out = low;
%   elseif in > high
%       out = high;
%   else
%       out = in;
%   end
% 
% % end sat  
% 
% 
