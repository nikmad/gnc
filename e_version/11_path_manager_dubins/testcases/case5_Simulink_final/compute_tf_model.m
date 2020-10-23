function [T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta, T_v_delta_r]...
    = compute_tf_model(x_trim, u_trim, y_trim, vtol)
% x_trim is the trimmed state,
% u_trim is the trimmed input

theta_trim = x_trim(8);

Va_trim = vtol.Va_trim;

alpha_trim  = y_trim(2);

delta_e_trim = u_trim(1);
delta_t_trim = u_trim(4);

C_p_0       = vtol.Gamma3 * vtol.C_ell_0        + vtol.Gamma4 * vtol.C_n_0;
C_p_beta    = vtol.Gamma3 * vtol.C_ell_beta     + vtol.Gamma4 * vtol.C_n_beta;
C_p_p       = vtol.Gamma3 * vtol.C_ell_p        + vtol.Gamma4 * vtol.C_n_p;
C_p_r       = vtol.Gamma3 * vtol.C_ell_r        + vtol.Gamma4 * vtol.C_n_r;
C_p_delta_a = vtol.Gamma3 * vtol.C_ell_delta_a  + vtol.Gamma4 * vtol.C_n_delta_a;
C_p_delta_r = vtol.Gamma3 * vtol.C_ell_delta_r  + vtol.Gamma4 * vtol.C_n_delta_r;

value1 = 0.5 * vtol.rho * Va_trim^2 * vtol.S_wing * vtol.b;
value2 = 0.5 * vtol.rho * Va_trim^2 * vtol.S_wing * vtol.c / vtol.Jy;
value3 = 0.5 * vtol.rho * Va_trim * vtol.S_wing / vtol.mass;

a_phi1 =  -value1 * C_p_p * vtol.b/(2*Va_trim);
a_phi2 =   value1 * C_p_delta_a;

a_theta1 = -value2 * vtol.C_m_q * vtol.c /(2*Va_trim);
a_theta2 = -value2 * vtol.C_m_alpha;
a_theta3 =  value2 * vtol.C_m_delta_e;

% Following transfer function coefficients for velocity have been updated
% using Errata
Vin = vtol.V_max * delta_t_trim;

%parameters of quadratic equation solution
%_________________________________________

% parameters defined to merely simplify the actual expression of Omega_p
a_omega = vtol.rho*(vtol.D_prop)^5*vtol.C_Q0/(2*pi)^2;
b_omega = vtol.rho*(vtol.D_prop)^4*vtol.C_Q1*Va_trim/(2*pi) + (vtol.KQ)^2/vtol.R_motor;
c_omega = vtol.rho*(vtol.D_prop)^3*vtol.C_Q2*Va_trim^2 - vtol.KQ * Vin/vtol.R_motor + vtol.KQ * vtol.i0;

Omega_p = (-b_omega + sqrt(b_omega^2-4*a_omega*c_omega))/(2*a_omega);

% parameters defined to merely simplify the actual expression of dTpVa and dTpDelta_t
vk1 = vtol.rho*(vtol.D_prop)^4*vtol.C_T0/(2*pi)^2;
vk2 = vtol.rho*(vtol.D_prop)^3*vtol.C_T1/(2*pi);
vk3 = vtol.rho*(vtol.D_prop)^2*vtol.C_T2;
vk4 = vtol.rho*(vtol.D_prop)^4*vtol.C_Q1/(2*pi);
vk5 = vtol.KQ^2 / vtol.R_motor;
vk6 = vtol.rho*(vtol.D_prop)^3 * vtol.C_Q2;
vk7 = -vtol.V_max * (vtol.KQ / vtol.R_motor);
vk8 = vtol.KQ * vtol.i0;
vk9 = (vk4 * Va_trim + vk5)^2 - 4*a_omega*(vk6 * Va_trim^2 + vk7 * delta_t_trim + vk8); % = b^2 - 4*a*c
vk10 = 2*(vk4*Va_trim + vk5)*vk4 - 4*a_omega*(2*vk6*Va_trim); % vk10 = d(vk9)/d(Va)

dOmegaVa = (1/(2*a_omega))*(-vk4 + 0.5 * (1/sqrt(vk9)) * vk10); % = d(Omega_p)/d(Va)

dTpVa       = 2 * vk1 * Omega_p * dOmegaVa + vk2*(Va_trim * dOmegaVa + Omega_p) + 2 * vk3 * Va_trim; % = d(T_p)/d(Va)
dTpDelta_t  = (-vk7/sqrt(vk9)) * (2*vk1*Omega_p + vk2*Va_trim); % = d(T_p)/d(delta_t)

a_V1 = (1/vtol.mass) * (vtol.rho * Va_trim * vtol.S_wing *(vtol.C_D_0 + ...
        vtol.C_D_alpha*alpha_trim + vtol.C_D_delta_e * delta_e_trim) - dTpVa); 
a_V2 = (1/vtol.mass) * dTpDelta_t;
a_V3 = vtol.gravity * cos(theta_trim - alpha_trim); 

a_beta1 = -value3 * vtol.C_Y_beta;
a_beta2 =  value3 * vtol.C_Y_delta_r;
    
% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([vtol.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([a_beta2],[1,a_beta1]);

end
