function plots(uu)

    % process inputs to function
    hx          = uu(1);       
    hy          = uu(2);       
    hz          = uu(3);       
    wx          = uu(4);       
    wy          = uu(5);       
    wz          = uu(6);       
    phi         = 180/pi*uu(7);
    theta       = 180/pi*uu(8);
    psi         = 180/pi*uu(9);
    tauX        = uu(10);     
    tauY        = uu(11);     
    tauZ        = uu(12);     
    t           = uu(13);     
    
    % define persistent variables 
    persistent hx_handle
    persistent hy_handle
    persistent hz_handle
    persistent wx_handle
    persistent wy_handle
    persistent wz_handle
    persistent phi_handle
    persistent theta_handle
    persistent psi_handle
    persistent tauX_handle
    persistent tauY_handle
    persistent tauZ_handle

  % first time function is called, initialize plot and persistent vars
    if t==0
        
        figure(1); 
        
        set(gcf,'color',[77/255,77/255,77/255]);
        
        subplot(3,4,2,'color',[0.275,0.509,0.706])
        hold on
        grid on
        hx_handle = graph_y(t, hx, [], 'g');
        title('h_x','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(3,4,6,'color',[0.275,0.509,0.706])
        hold on
        grid on
        hy_handle = graph_y(t, hy, [], 'g');
        title('h_y','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(3,4,10,'color',[0.275,0.509,0.706])
        hold on
        grid on
        hz_handle = graph_y(t, hz, [], 'g');
        title('h_z','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);

        subplot(3,4,3,'color',[0.275,0.509,0.706])
        hold on
        grid on
        wx_handle = graph_y(t, wx, [], 'g');
        title('\omega_x','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);

        subplot(3,4,7,'color',[0.275,0.509,0.706])
        hold on
        grid on
        wy_handle = graph_y(t, wy, [], 'g');
        title('\omega_y','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);

        subplot(3,4,11,'color',[0.275,0.509,0.706])
        hold on
        grid on
        wz_handle = graph_y(t, wz, [], 'g');
        title('\omega_z','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);

        subplot(3,4,4,'color',[0.275,0.509,0.706])
        hold on
        grid on
        phi_handle = graph_y(t, phi, [], 'g');
        title('\phi','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(3,4,8,'color',[0.275,0.509,0.706])
        hold on
        grid on
        theta_handle = graph_y(t, theta, [], 'g');
        title('\theta','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(3,4,12,'color',[0.275,0.509,0.706])
        hold on
        grid on
        psi_handle = graph_y(t, psi, [], 'g');
        title('\psi','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(3,4,1,'color',[0.275,0.509,0.706])
        hold on
        grid on
        tauX_handle = graph_y(t, tauX, [], 'g');
        title('\tau_x','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(3,4,5,'color',[0.275,0.509,0.706])
        hold on
        grid on
        tauY_handle = graph_y(t, tauY, [], 'g');
        title('\tau_y','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
        subplot(3,4,9,'color',[0.275,0.509,0.706])
        hold on
        grid on
        tauZ_handle = graph_y(t, tauZ, [], 'g');
        title('\tau_z','fontweight','bold', 'fontsize', 20, 'color', [255/255,179/255,25/255]);
        
    % at every other time step, redraw state variables
    else 
       graph_y(t, hx, hx_handle);
       graph_y(t, hy, hy_handle);
       graph_y(t, hz, hz_handle);
       graph_y(t, wx, wx_handle);
       graph_y(t, wy, wy_handle);
       graph_y(t, wz, wz_handle);
       graph_y(t, phi, phi_handle);
       graph_y(t, theta, theta_handle);
       graph_y(t, psi, psi_handle);
       graph_y(t, tauX, tauX_handle);
       graph_y(t, tauY, tauY_handle);
       graph_y(t, tauZ, tauZ_handle);
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = graph_y(t, y, handle, color)
  if isempty(handle)
    handle    = plot(t,y,color,'LineWidth', 2);
    set(gca,'xcolor','w', 'ycolor','w');
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
  end
