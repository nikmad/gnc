function plotvtolstatevariables(uu)

    % process inputs to function
    pn          = uu(1);             % North position (meters)
    pe          = uu(2);             % East position (meters)
    h           = -uu(3);            % altitude (meters)
    u           = uu(4);             % body velocity along x-axis (meters/s)
    v           = uu(5);             % body velocity along y-axis (meters/s)
    w           = uu(6);             % body velocity along z-axis (meters/s)
    phi         = 180/pi*uu(7);      % roll angle (degrees)   
    theta       = 180/pi*uu(8);      % pitch angle (degrees)
    psi         = uu(9);             % yaw angle (radians)
    p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
    q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
    r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)
    Va          = uu(13);            % airspeed (m/s)
    alpha       = 180/pi*uu(14);     % angle of attack (degrees)
    beta        = 180/pi*uu(15);     % side slip angle (degrees)
    wn          = uu(16);            % wind in the North direction
    we          = uu(17);            % wind in the East direction
    wd          = uu(18);            % wind in the Down direction
    pn_c        = uu(19);            % commanded North position (meters)
    pe_c        = uu(20);            % commanded East position (meters)
    h_c         = uu(21);            % commanded altitude (meters)
    Va_c        = uu(22);            % commanded airspeed (meters/s)
    alpha_c     = 180/pi*uu(23);     % commanded angle of attack (degrees)
    beta_c      = 180/pi*uu(24);     % commanded side slip angle (degrees)
    phi_c       = 180/pi*uu(25);     % commanded roll angle (degrees)   
    theta_c     = 180/pi*uu(26);     % commanded pitch angle (degrees)
    chi_c       = 180/pi*uu(27);     % commanded course (degrees)
    p_c         = 180/pi*uu(28);     % commanded body angular rate along x-axis (degrees/s)
    q_c         = 180/pi*uu(29);     % commanded body angular rate along y-axis (degrees/s)
    r_c         = 180/pi*uu(30);     % commanded body angular rate along z-axis (degrees/s)
    pn_hat      = uu(31);            % estimated North position (meters)
    pe_hat      = uu(32);            % estimated East position (meters)
    h_hat       = uu(33);            % estimated altitude (meters)
    Va_hat      = uu(34);            % estimated airspeed (meters/s)
    alpha_hat   = 180/pi*uu(35);     % estimated angle of attack (degrees)
    beta_hat    = 180/pi*uu(36);     % estimated side slip angle (degrees)
    phi_hat     = 180/pi*uu(37);     % estimated roll angle (degrees)   
    theta_hat   = 180/pi*uu(38);     % estimated pitch angle (degrees)
    chi_hat     = 180/pi*uu(39);     % estimated course (degrees)
    p_hat       = 180/pi*uu(40);     % estimated body angular rate along x-axis (degrees/s)
    q_hat       = 180/pi*uu(41);     % estimated body angular rate along y-axis (degrees/s)
    r_hat       = 180/pi*uu(42);     % estimated body angular rate along z-axis (degrees/s)
%    Vg_hat      = uu(43);            % estimated groundspeed
%    wn_hat      = uu(44);            % estimated North wind
%    we_hat      = uu(45);            % estimated East wind
%    psi_hat     = 180/pi*uu(46);     % estimated heading
%    bx_hat      = uu(47);            % estimated x-gyro bias
%    by_hat      = uu(48);            % estimated y-gyro bias
%    bz_hat      = uu(49);            % estimated z-gyro bias
    delta_e     = 180/pi*uu(50);     % elevator angle (degrees)
    delta_a     = 180/pi*uu(51);     % aileron angle (degrees)
    delta_r     = 180/pi*uu(52);     % rudder angle (degrees)
    delta_t     = uu(53);            % throttle setting (unitless)
    t           = uu(54);            % simulation time
    
    % compute course angle
    %chi = 180/pi*atan2(Va*sin(psi)+we, Va*cos(psi)+wn);
    chi = chi_hat;
    %Va = 50;
    V_wind = sqrt(wn^2+we^2+wd^2);

%     gamma_a = theta-alpha;
%     gamma_a_c = theta_c-alpha_c;
%     gamma_a_hat = theta_hat-alpha_hat;
    
    % define persistent variables 
    persistent pn_handle
    persistent pe_handle
    persistent h_handle
    persistent Va_handle
    %persistent gamma_a_handle
    persistent alpha_handle
    persistent beta_handle
    persistent phi_handle
    persistent theta_handle
    persistent chi_handle
    persistent psi_handle
    persistent p_handle
    persistent q_handle
    persistent r_handle
    persistent delta_e_handle
    persistent delta_a_handle
    persistent delta_r_handle
    persistent delta_t_handle
    persistent wind_handle 
    persistent Vwind_handle
    persistent uvw_handle
    

  % first time function is called, initialize plot and persistent vars
    if t==0
        
        figure(2), clf
        
        set(gcf,'color',[77/255,77/255,77/255]);
        %figure('color',[0.47,0.53,0.6]);
        
        subplot(4,5,4,'color',[0.275,0.509,0.706])
        hold on
        grid on
        pn_handle = graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', []);
        

        subplot(4,5,2,'color',[0.275,0.509,0.706])
        hold on
        grid on
        Va_handle = graph_y_yhat_yd(t, Va, Va_hat, Va_c, 'V_a', []);
        %gamma_a_handle = graph_y_yhat_yd(t, gamma_a, gamma_a_hat, gamma_a_c, '(\gamma)_a', []);

        subplot(4,5,14,'color',[0.275,0.509,0.706])
        hold on
        grid on
        pe_handle = graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', []);

        subplot(4,5,3,'color',[0.275,0.509,0.706])
        hold on
        grid on
        alpha_handle = graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', []);

        subplot(4,5,9,'color',[0.275,0.509,0.706])
        hold on
        grid on
        h_handle = graph_y_yhat_yd(t, h, h_hat, h_c, 'h', []);

        subplot(4,5,15,'color',[0.275,0.509,0.706])
        hold on
        grid on
        beta_handle = graph_y_yhat_yd(t, beta, beta_hat, beta_c, '\beta', []);

        subplot(4,5,18,'color',[0.275,0.509,0.706])
        hold on
        grid on
        phi_handle = graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi', []);
        
        subplot(4,5,17,'color',[0.275,0.509,0.706])
        hold on
        grid on
        p_handle = graph_y_yhat_yd(t, p, p_hat, p_c, 'p', []);
        
        subplot(4,5,8,'color',[0.275,0.509,0.706])
        hold on
        grid on
        theta_handle = graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', []);
        
        subplot(4,5,7,'color',[0.275,0.509,0.706])
        hold on
        grid on
        q_handle = graph_y_yhat_yd(t, q, q_hat, q_c, 'q', []);
        
        subplot(4,5,19,'color',[0.275,0.509,0.706])
        hold on
        grid on
        chi_handle = graph_y_yhat_yd(t, chi, chi_hat, chi_c, '\chi', []);
        
        subplot(4,5,12,'color',[0.275,0.509,0.706])
        hold on
        grid on
        r_handle = graph_y_yhat_yd(t, r, r_hat, r_c, 'r', []);
        
      
        subplot(4,5,6,'color',[0.275,0.509,0.706])
        hold on
        grid on
        delta_e_handle = graph_y(t, delta_e, [], 'y');
        title('\delta_e','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(4,5,16,'color',[0.275,0.509,0.706])
        hold on
        grid on
        delta_a_handle = graph_y(t, delta_a, [], 'y');
        title('\delta_a','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);

        subplot(4,5,11,'color',[0.275,0.509,0.706])
        hold on
        grid on
        delta_r_handle = graph_y(t, delta_r, [], 'y');
        title('\delta_r','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(4,5,1,'color',[0.275,0.509,0.706])
        hold on
        grid on
        delta_t_handle = graph_y(t, delta_t, [], 'y');
        title('\delta_t','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(4,5,13,'color',[0.275,0.509,0.706])
        hold on
        grid on
        psi_handle = graph_y_yhat_yd(t, psi, psi, psi, '\psi', []);
        
        subplot(4,5,10,'color',[0.275,0.509,0.706])
        hold on
        grid on
        wind_handle = graph_y_yhat_yd(t, wn, we, wd, 'wn,we,wd', []);
        
        subplot(4,5,5,'color',[0.275,0.509,0.706])
        hold on
        grid on
        Vwind_handle = graph_y_yhat_yd(t, V_wind, V_wind, V_wind, 'Vwind', []);
        
        subplot(4,5,20,'color',[0.275,0.509,0.706])
        hold on
        grid on
        uvw_handle = graph_y_yhat_yd(t, u, v, w, 'uvw', []);
        
    % at every other time step, redraw state variables
    else 
       
       graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n\t', pn_handle);
       graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e\t', pe_handle);
       graph_y_yhat_yd(t, h, h_hat, h_c, 'h\t', h_handle);
       graph_y_yhat_yd(t, Va, Va_hat, Va_c, 'V_a\t', Va_handle);
       %graph_y_yhat_yd(t, gamma_a, gamma_a_hat, gamma_a_c, '(\gamma)_a', gamma_a_handle);
       graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha\t', alpha_handle);
       graph_y_yhat_yd(t, beta, beta_hat, beta_c, '\beta\t', beta_handle);
       graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi\t', phi_handle);
       graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta\t', theta_handle);
       graph_y_yhat_yd(t, chi, chi_hat, chi_c, '\chi\t', chi_handle);
       graph_y_yhat_yd(t, psi, psi, psi, '\psi\t', psi_handle);
       graph_y_yhat_yd(t, wn, we, wd, 'wn,we,wd\t', wind_handle);
       graph_y_yhat_yd(t, V_wind, V_wind, V_wind, 'Vwind\t', Vwind_handle);
       graph_y_yhat_yd(t, u, v, w, 'uvw\t', uvw_handle);
       graph_y_yhat_yd(t, p, p_hat, p_c, 'p\t', p_handle);
       graph_y_yhat_yd(t, q, q_hat, q_c, 'q\t', q_handle);
       graph_y_yhat_yd(t, r, r_hat, r_c, 'r\t', r_handle);
       graph_y(t, delta_e, delta_e_handle);
       graph_y(t, delta_a, delta_a_handle);
       graph_y(t, delta_r, delta_r_handle);
       graph_y(t, delta_t, delta_t_handle);
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, handle, color)
  
  if isempty(handle)
    handle    = plot(t,y,color,'LineWidth', 2);
    %title('y','fontweight','bold', 'fontsize', 20, 'color', [240/255,230/255,140/255]);
    set(gca,'xcolor','w', 'ycolor','w');
    %legend('Location','northeast')
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle)
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'g--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  
  if isempty(handle)
    
    %handle(2)   = plot(t,yhat,'d', 'color', [1 1 0],'DisplayName','\color{white} yhat');
    handle(1)   = plot(t,y,'color', [255/255,215/255,0]);
    handle(2)   = plot(t,yhat, 'color', [0,1,0],'LineWidth', 2);
    handle(3)   = plot(t,yd,'color',[205/255,92/255,92/255], 'LineWidth', 2);
    title(lab,'fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
    %ylabel(lab,
    set(get(gca,'YLabel'),'Rotation',0.0);
    set(gca,'xcolor','w', 'ycolor','w');
    %legend('Location','southeast')
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
    %drawnow
  end

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low
      out = low;
  elseif in > high
      out = high;
  else
      out = in;
  end

% end sat  


