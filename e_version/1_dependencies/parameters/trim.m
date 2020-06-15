%function [alpha_trim,beta_trim,delta_e_trim,delta_a_trim,delta_r_trim,Tx_trim,stall_speed]= Autopilot(aero_data,structure_data,V_trim,propulsion_data, atmos_data)
% b1=propulsion_data(1); % b and d of vtol propellers
% d1=propulsion_data(2);
% b1_inv=1.0/b1;

%rho=atmos_data(4); % density
rho=1.225;

V=25;
%mass=structure_data(1,1);s=structure_data(1,4);
mass = 11;
s = .55;



% CLo =aero_data(1,1); CL_alpha=aero_data(1,2);CL_beta=aero_data(1,3); CL_delta_e=aero_data(1,4); CL_delta_a=aero_data(1,5); CL_delta_r=aero_data(1,6); CL_p=aero_data(1,7);CL_q=aero_data(1,8);CL_r=aero_data(1,9);
% 
% Cdo =aero_data(2,1); Cd_alpha=aero_data(2,2);Cd_beta=aero_data(2,3); Cd_delta_e=aero_data(2,4); Cd_delta_a=aero_data(2,5); Cd_delta_r=aero_data(2,6); Cd_p=aero_data(2,7);Cd_q=aero_data(2,8);Cd_r=aero_data(2,9);
% 
% CYo =aero_data(3,1); CY_alpha=aero_data(3,2);CY_beta=aero_data(3,3); CY_delta_e=aero_data(3,4); CY_delta_a=aero_data(3,5); CY_delta_r=aero_data(3,6); CY_p=aero_data(3,7);CY_q=aero_data(3,8);CY_r=aero_data(3,9);
% 
% Clo =aero_data(4,1); Cl_alpha=aero_data(4,2);Cl_beta=aero_data(4,3); Cl_delta_e=aero_data(4,4); Cl_delta_a=aero_data(4,5); Cl_delta_r=aero_data(4,6); Cl_p=aero_data(4,7);Cl_q=aero_data(4,8);Cl_r=aero_data(4,9);
% 
% Cmo =aero_data(5,1); Cm_alpha=aero_data(5,2);Cm_beta=aero_data(5,3); Cm_delta_e=aero_data(5,4); Cm_delta_a=aero_data(5,5); Cm_delta_r=aero_data(5,6); Cm_p=aero_data(5,7);Cm_q=aero_data(5,8);Cm_r=aero_data(5,9);
% 
% Cno =aero_data(6,1); Cn_alpha=aero_data(6,2);Cn_beta=aero_data(6,3); Cn_delta_e=aero_data(6,4); Cn_delta_a=aero_data(6,5); Cn_delta_r=aero_data(6,6); Cn_p=aero_data(6,7);Cn_q=aero_data(6,8);Cn_r=aero_data(6,9);

CLo=.23;
Cmo=.0135;
CL_alpha = 5.61;
CL_delta_e = .13;
Cm_alpha = -2.74;
Cm_delta_e = -0.99;
Cdo = .043;
Cd_alpha = .03;
Cd_delta_e = .0135;


g=9.81;



tb=[(mass*g/(0.5*rho*V*V*s)) - CLo;-Cmo];
ta=[CL_alpha,CL_delta_e;Cm_alpha,Cm_delta_e];

trim_set_long = inv(ta)*tb;

alpha_trim = trim_set_long(1);
delta_e_trim = trim_set_long(2);

% CYo + CY_beta*beta + CY_delta_a*delta_a + CY_delta_r*delta_r;
% Clo + Cl_beta*beta + Cl_delta_a*delta_a + Cl_delta_r*delta_r;
% Cno + Cn_beta*beta + Cn_delta_a*delta_a + Cn_delta_r*delta_r;

% tb1=[-CYo;-Clo;-Cno];
% ta1=[CY_beta,CY_delta_a,CY_delta_r;Cl_beta,Cl_delta_a,Cl_delta_r;Cn_beta,Cn_delta_a,Cn_delta_r];
% 
% trim_set_lat=inv(ta1)*tb1;
% 
% beta_trim = trim_set_lat(1);
% delta_a_trim = trim_set_lat(2);
% delta_r_trim = trim_set_lat(3);


% drag = 1/2 *rho *V*V*s*( Cdo + Cd_alpha*alpha + Cd_delta_e*delta_e );
Tx_trim =  0.5*rho*V*V*s*( Cdo + Cd_alpha*alpha_trim + Cd_delta_e*delta_e_trim );

stall_speed=sqrt((mass*g)/(0.5*1.225*s*(CLo+CL_alpha*(12.0/57.3))));
