
function out = forces_moments(x, delta, wind, P)

   
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    Mass    = P.mass;  % Mass of aircraft
    g       = P.gravity;% gravity 
    rho     = P.rho;    % Density
    S       = P.S_wing; % Area
    
    % aerodynamic coefficients
    b           = P.b;%             = 2.8956;
    c           = P.c;%             = 0.18994;
    S_prop      = P.S_prop;  %      = 0.2027;
    K_motor     = P.k_motor;%       = 80;
    Ktp         = P.k_T_P;%         = 0;
    K_omega     = P.k_Omega;%       = 0;
    eff         = P.e;%             = 0.9;

    CL0         = P.C_L_0;%         = 0.28;
%     CL_alpha    = P.C_L_alpha;%     = 3.45;
    CLq         = P.C_L_q;%         = 0.0;
    CL_delta_e  = P.C_L_delta_e;%   = -0.36;
    CD0         = P.C_D_0;%         = 0.03;
%     CD_alpha    = P.C_D_alpha;%     = 0.30;
    CDp         = P.C_D_p;%         = 0.0437;
    CDq         = P.C_D_q;%         = 0.0;
    CD_delta_e  = P.C_D_delta_e;%   = 0.0;
    Cm0         = P.C_m_0;%         = -0.02338;
    Cm_alpha    = P.C_m_alpha;%     = -0.38;
    Cmq         = P.C_m_q;%         = -3.6;
    Cm_delta_e  = P.C_m_delta_e;%   = -0.5;
    CY0         = P.C_Y_0;%         = 0.0;
    CY_beta     = P.C_Y_beta;%      = -0.98;
    CYp         = P.C_Y_p;%         = 0.0;
    CYr         = P.C_Y_r;%         = 0.0;
    CY_delta_a  = P.C_Y_delta_a;%   = 0.0;
    CY_delta_r  = P.C_Y_delta_r;%   = -0.17;
    Cl0         = P.C_ell_0;%       = 0.0;
    Cl_beta     = P.C_ell_beta;%    = -0.12;
    Clp         = P.C_ell_p;%       = -0.26;
    Clr         = P.C_ell_r;%       = 0.14;
    Cl_delta_a  = P.C_ell_delta_a;% = 0.08;
    Cl_delta_r  = P.C_ell_delta_r;% = 0.105;
    Cn0         = P.C_n_0;%         = 0.0;
    Cn_beta     = P.C_n_beta;%      = 0.25;
    Cnp         = P.C_n_p;%         = 0.022;
    Cnr         = P.C_n_r;%         = -0.35;
    Cn_delta_a  = P.C_n_delta_a;%   = 0.06;
    Cn_delta_r  = P.C_n_delta_r;%   = -0.032;
    C_prop      = P.C_prop;%        = 1.0;
    P.M         = 50;
    P.epsilon   = 0.1592;
    P.alpha0    = 0.4712;

    % compute wind data in NED
    Rvb=[cos(theta)*cos(psi) cos(theta)*sin(phi) -sin(theta)
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)  sin(phi)*cos(theta)
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)  cos(phi)*cos(theta)];
    
    
    % compute wind vector in the inertial frame
    w_n = w_ns + Rvb(:,1)'*[u_wg; v_wg; w_wg];
    w_e = w_es + Rvb(:,2)'*[u_wg; v_wg; w_wg];
    w_d = w_ds + Rvb(:,3)'*[u_wg; v_wg; w_wg];
%     [wind]=Rvb*[w_n;w_e;w_d];
    % compute air data
    u_r   = u-(w_n+u_wg);
    v_r   = v-(w_e+v_wg);
    w_r   = w-(w_d+w_wg);
    Va    = sqrt(u_r^2+v_r^2+w_r^2);%0;
    alpha = atan(w_r/u_r);%0;
    beta  = asin(v_r/Va);%0;
    
    %---------Computation of Cx_alpha,Cxq_alpha &Cx_dE_alpha---------%
        sigma_alpha=(1+exp(-P.M*(alpha-P.alpha0))+exp(P.M*(alpha+P.alpha0)))/...
                ((1+exp(-P.M*(alpha-P.alpha0)))*(1+exp(P.M*(alpha+P.alpha0))));
    CL_alpha=(1-sigma_alpha)*(P.C_L_0+P.C_L_alpha*alpha)+...
         sigma_alpha*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    CD_alpha=P.C_D_p+(P.C_L_0+P.C_L_alpha*alpha)^2/(pi*P.e*(P.b^2/P.S_wing));
    Cx_alpha   = -(CD_alpha)*cos(alpha)+(CL_alpha)*sin(alpha);
    Cxq_alpha  = -CDq*cos(alpha)+CLq*sin(alpha);
    Cx_dE_alpha= -CL_delta_e*cos(alpha)+CL_delta_e*sin(alpha);
    
    
    %---------Computation of Cz_alpha,Czq_alpha &Cz_dE_alpha---------%
    
    Cz_alpha   = -(CD_alpha)*sin(alpha)-(CL_alpha)*cos(alpha);
    Czq_alpha  = -CDq*sin(alpha)-CLq*cos(alpha);
    Cz_dE_alpha= -CD_delta_e*sin(alpha)-CL_delta_e*cos(alpha);
    
    % compute external forces and torques on aircraft
    Q_const  = 0.5*rho*Va*Va*S;
    Force(1) = -Mass*g*sin(theta)+Q_const*(Cx_alpha+Cxq_alpha*(c*q/(2*Va))+Cx_dE_alpha*delta_e)+(0.5*rho*S_prop*C_prop*(K_motor*K_motor*delta_t*delta_t-Va*Va));%0;
    Force(2) =  Mass*g*cos(theta)*sin(phi)+Q_const*(CY0+CY_beta*beta+(CYp*b*p/2/Va)+(CYr*b*r/2/Va)+CY_delta_a*delta_a+CY_delta_r*delta_r);%0;
    Force(3) =  Mass*g*cos(theta)*cos(phi)+Q_const*(Cz_alpha+(Czq_alpha*c*q/2/Va)+Cz_dE_alpha*delta_e);%0;
    
    Moment_A  = [b*(Cl0+Cl_beta*beta+Clp*b*p/2/Va+Clr*b*r/2/Va+Cl_delta_a*delta_a+Cl_delta_r*delta_r);
        c*(Cm0+Cm_alpha*alpha+Cmq*c*q/2/Va+Cm_delta_e*delta_e);
        b*(Cn0+Cn_beta*beta+Cnp*b*p/2/Va+Cnr*b*r/2/Va+Cn_delta_a*delta_a+Cn_delta_r*delta_r)];
    Moment_T  = [-Ktp*K_omega*delta_t*K_omega*delta_t;0;0];
    Net_Moment= Q_const*Moment_A+Moment_T;          
    

    Torque(1) = Net_Moment(1);
    Torque(2) = Net_Moment(2);   
    Torque(3) = Net_Moment(3);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



