
delta_t = 1.07;
rho = 1.225;
Va = 35;
% Parameters for propulsion thrust and torque models
	vtol.D_prop = 0.508;     % prop diameter in m

	% Motor parameters
	vtol.K_V = 145;                    % from datasheet RPM/V
	%vtol.KQ = (1/vtol.K_V)*60/(2*pi);   % KQ in N-m/A, V-s/rad
    vtol.KQ = 0.0659; % Corrected using Errata
	vtol.R_motor = 0.042;              % ohms
	vtol.i0 = 1.5;                     % no-load (zero-torque) current (A)

	% Inputs
	vtol.ncells = 12;
	vtol.V_max = 3.7*vtol.ncells;       % max voltage for specified number of battery cells

	% Coeffiecients from prop_data fit
	vtol.C_Q2 = -0.01664;
	vtol.C_Q1 = 0.004970;
	vtol.C_Q0 = 0.005230;

	vtol.C_T2 = -0.1079;
	vtol.C_T1 = -0.06044;
	vtol.C_T0 = 0.09357;    

    Vin = vtol.V_max * delta_t;

    a_omega = rho*(vtol.D_prop)^5*vtol.C_Q0/(2*pi)^2;
    b_omega = rho*(vtol.D_prop)^4*vtol.C_Q1*Va/(2*pi) + (vtol.KQ)^2/vtol.R_motor;
    c_omega = rho*(vtol.D_prop)^3*vtol.C_Q2*Va^2 - vtol.KQ * Vin/vtol.R_motor + vtol.KQ * vtol.i0;
    
    Omega_p = (-b_omega + sqrt(b_omega^2-4*a_omega*c_omega))/(2*a_omega);
    %Omega_p = 1.0;
    
    
    Tp = rho*(vtol.D_prop)^4*vtol.C_T0*Omega_p^2/(4*pi^2)+ ...
          rho*(vtol.D_prop)^3*vtol.C_T1*Va*Omega_p/(2*pi) + ...
          rho*(vtol.D_prop)^2*vtol.C_T2*Va^2
      
      