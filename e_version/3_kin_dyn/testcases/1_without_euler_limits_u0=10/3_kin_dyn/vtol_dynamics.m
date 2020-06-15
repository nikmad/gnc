% nkm - "This file is the modified version of initially written file "vtol_dynamics_quaternions.m"...
% While this file is in Euler representation, initial file was in quaternions. The reason I had to 
% make this modified file is that in chapter 5 while using trim algorithm (in compute_trim.m file), dx0 must have the same 
% dimensions as the number of continuous states (sizes.NumContStates) as defined in this file here.
% In case of quaternions, we need 4 variables (e0,e1,e2,e3) where as in Euler we need only three 
%(phi, theta, psi). Since trim algorithm in chapter 5 is given in Euler angles, dx0 is created with
% only 12 states where as in this file if it is written in quaternions, it will have 12 continuous 
% states and thus compute_trim.m algorithm throws errors. To circumvent this problem, I had to 
% rewrite this file again in Euler so that both here and in compute_trim, the number of states
% will be only 12 and thereby compute_trim algorithm works without issues."

function [sys,x0,str,ts,simStateCompliance] = vtol_dynamics(t,x,u,flag,vtol)

switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(vtol);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u,vtol);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl
end

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(vtol)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;
%sizes.NumContStates  = 13; % for Quaternion formulae
sizes.NumContStates  = 12; % for Euler angle formulae
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    vtol.pn0;...
    vtol.pe0;...
    vtol.pd0;...
    vtol.u0;...
    vtol.v0;...
    vtol.w0;...
    %vtol.e0;...
    %vtol.e1;...
    %vtol.e2;...
    %vtol.e3;...
    vtol.phi0;...
    vtol.theta0;...
    vtol.psi0;...
    vtol.p0;...
    vtol.q0;...
    vtol.r0;...
    ];

% x0 = [...
%     -15.9778876062230;
%     81.3354866104541;
%     27744.0595938172;
%     -5.00000000000000e-10;
%     -3.05450624818481e-26;
%     1.03535308846351e-25;
%     -3.85438045205601;
%     -4.71238898038469;
%     -14.8033041363566;
%     -6.35756674525291e-09;
%     -8.27374480657324e-09;
%     -2.57525187874944e-10
%     ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes
end

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, vtol)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
%     e0    = x(7);
%     e1    = x(8);
%     e2    = x(9);
%     e3    = x(10);
    phi = x(7);
    theta = x(8);
    psi = x(9);
%     p     = x(11);
%     q     = x(12);
%     r     = x(13);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    
% Data for Aerosonde UAV
    mass = vtol.mass;
    Ix = vtol.Jx;
    Iy = vtol.Jy;
    Iz = vtol.Jz;
    Ixz= vtol.Jxz;
    %Ixz = 0;

    c0=vtol.Gamma; 
    c1=vtol.Gamma1;    
    c2=vtol.Gamma2;                                              
    c3=vtol.Gamma3;
    c4=vtol.Gamma4;
    c5=vtol.Gamma5;
    c6=vtol.Gamma6;
    c7=vtol.Gamma7;
    c8=vtol.Gamma8;

%     pndot = (e1^2+e0^2-e2^2-e3^2)*u + 2*(e1*e2-e3*e0)*v + 2*(e1*e3+e2*e0)*w;
%     pedot = 2*(e1*e2+e3*e0)*u + (e2^2+e0^2-e1^2-e3^2)*v + 2*(e2*e3-e1*e0)*w;
%     pddot = 2*(e1*e3-e2*e0)*u + 2*(e2*e3+e1*e0)*v + (e3^2+e0^2-e1^2-e2^2)*w;
    
    pndot = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)) ...
        + w*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
    pedot = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)) ...
        + w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
    pddot = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta);

    udot = r*v-q*w + fx/mass;
    vdot = p*w-r*u + fy/mass;
    wdot = q*u-p*v + fz/mass;
       
%     e0dot = 0.5*(0*e0-p*e1-q*e2-r*e3);
%     e1dot = 0.5*(p*e0+0*e1+r*e2-q*e3);
%     e2dot = 0.5*(q*e0-r*e1+0*e2+p*e3);
%     e3dot = 0.5*(r*e0+q*e1-p*e2+0*e3);

	phidot = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
	thetadot = q*cos(phi)-r*sin(phi);
	psidot = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
        
    pdot = c1*p*q-c2*q*r + c3*ell+c4*n;
    qdot = c5*p*r-c6*(p^2-r^2) + m/Iy;
    rdot = c7*p*q-c1*q*r + c4*ell+c8*n;
        

%sys = [pndot; pedot; pddot; udot; vdot; wdot; e0dot; e1dot; e2dot; e3dot; pdot; qdot; rdot];
sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

% end mdlDerivatives
end

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,uu)

sys = [];

% end mdlUpdate
end

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x)
%     e0    = x(7);
%     e1    = x(8);
%     e2    = x(9);
%     e3    = x(10);

%     y(1) = x(1);
%     y(2) = x(2);
%     y(3) = x(3);
%     y(4) = x(4);
%     y(5) = x(5);
%     y(6) = x(6);

%     y(7) = atan2(2*(e0*e1+e2*e3),e0^2+e3^2-e1^2-e2^2);
%     y(8) = asin(2*(e0*e2-e1*e3));
%     y(9) = atan2(2*(e0*e3+e1*e2),e0^2+e1^2-e2^2-e3^2);

%     y(10) = x(11);
%     y(11) = x(12);
%     y(12) = x(13);

y = x;

sys = y;

% end mdlOutputs
end

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit
end

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
end
