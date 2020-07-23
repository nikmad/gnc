function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% mdlInitializeSizes

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.

sizes = simsizes;

sizes.NumContStates  = 12;
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
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.phi0;...
    P.theta0;...
    P.psi0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];


simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, P)

    pn    = x(1);
    pe    = x(2);
    pe    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
  %------Inertia constants-----------------%

  tau=P.Jx*P.Jz-(P.Jxz*P.Jxz);
  tau1=P.Jxz*(P.Jx-P.Jy+P.Jz)/tau;
  tau2=(P.Jz*(P.Jz-P.Jy)+P.Jxz*P.Jxz)/tau;
  tau3=P.Jz/tau;
  tau4=P.Jxz/tau;
  tau5=(P.Jz-P.Jx)/P.Jy;
  tau6=P.Jxz/P.Jy;
  tau7=((P.Jx-P.Jy)*P.Jx+P.Jxz*P.Jxz)/tau;
  tau8=P.Jx/tau;
  
  %~~~~~~~~~~~~~~~~~~~~~~~~~~Matrices~~~~~~~~~~~~~~~~~~~~~~%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  neddot=[cos(theta)*cos(psi), ...
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
        cos(theta)*sin(psi),...
        sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),...
        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
        -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)]*[u;v;w];
    
    veldot=[r*v-q*w;p*w-r*u;q*u-p*v]+1/P.mass*[fx;fy;fz];
    
    attdot=[1,sin(phi)*tan(theta),cos(phi)*tan(theta);
            0,cos(phi),-sin(phi);
            0,sin(phi)/cos(theta),cos(phi)/cos(theta)]*[p;q;r];
        
    attratdot=[tau1*p*q-tau2*q*r;
               tau5*p*r-tau6*(p^2-r^2);
               tau7*p*q-tau1*q*r] +...
              [tau3*ell+tau4*n;
               1/P.Jy*m;
               tau4*ell+tau8*n];
  
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %----Diferential Equations------%
    pndot = neddot(1);
    pedot = neddot(2);
    pddot = neddot(3);
    udot = veldot(1);
    vdot = veldot(2);
    wdot = veldot(3);
    phidot = attdot(1);
    thetadot = attdot(2);
    psidot = attdot(3);
    pdot = attratdot(1);
    qdot = attratdot(2);
    rdot = attratdot(3);
  

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];


% mdlUpdate
function sys=mdlUpdate(t,x,u)

sys = [];

% mdlOutputs

function sys=mdlOutputs(t,x,u)

sys = x;



function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];


