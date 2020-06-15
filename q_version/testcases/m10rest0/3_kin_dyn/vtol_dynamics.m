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

sizes.NumContStates  = 13;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 18;
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
    vtol.e0;...
    vtol.e1;...
    vtol.e2;...
    vtol.e3;...
    vtol.p0;...
    vtol.q0;...
    vtol.r0;...
    ];

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
function sys=mdlDerivatives(t,x,uu,vtol)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    e0    = x(7);
    e1    = x(8);
    e2    = x(9);
    e3    = x(10);
    p     = x(11);
    q     = x(12);
    r     = x(13);

    fx    = uu(13);
    fy    = uu(14);
    fz    = uu(15);
    ell   = uu(16);
    m     = uu(17);
    n     = uu(18);
    
    xprev = struct(...
      'a1', uu(1),... 
      'a2', uu(2),... 
      'a3', uu(3),... 
      'a4', uu(4),... 
      'a5', uu(5),... 
      'a6', uu(6),... 
      'a7', uu(7),... 
      'a8', uu(8),... 
      'a9', uu(9),... 
      'a10', uu(10),... 
      'a11', uu(11),... 
      'a12', uu(12)... 
      );
    

%{
    xprev(1) = uu(1);
    xprev(2) = uu(2);
    xprev(3) = uu(3);
    xprev(4) = uu(4);
    xprev(5) = uu(5);
    xprev(6) = uu(6);
    xprev(7) = uu(7);
    xprev(8) = uu(8);
    xprev(9) = uu(9);
    xprev(10) = uu(10);
    xprev(11) = uu(11);
    xprev(12) = uu(12);
    %}

    save('xprev.mat','xprev');

    %{
    fileID = fopen('xprev1.txt','a');
    
    %printf(fileID,'%12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s\n',...
    %  'xprev(1)','xprev(2)','xprev(3)','xprev(4)','xprev(5)','xprev(6)','xprev(7)',...
     % 'xprev(8)','xprev(9)','xprev(10)','xprev(11)','xprev(12)');
    fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n',xprev);
    fclose(fileID);


    %fileID=fopen('xprev1.txt','r');
    formatSpec = '%s', 'whitespace', '%s', 'whitespace', '%s', 'whitespace', '%s', 'whitespace', '%s', 'whitespace', ...
    '%s', 'whitespace', '%s', 'whitespace', '%s', 'whitespace', '%s', 'whitespace', '%s', 'whitespace', ...
    '%s', 'whitespace', '%s', 'whitespace\n';
    M=textread('xprev1.txt',formatSpec);
    %fclose(fileID);
    p_idx = size(M);
    %p_idx=logical(p_idx);
    
    
    fileID = fopen('xprev2.txt','a');
    
    %printf(fileID,'%12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s\n',...
    %  'xprev(1)','xprev(2)','xprev(3)','xprev(4)','xprev(5)','xprev(6)','xprev(7)',...
     % 'xprev(8)','xprev(9)','xprev(10)','xprev(11)','xprev(12)');
    %fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n',M);
    fprintf(fileID,'%3f,%3f\n',p_idx);
    fclose(fileID);
    %}



 
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

    pndot = (e1^2+e0^2-e2^2-e3^2)*u + 2*(e1*e2-e3*e0)*v + 2*(e1*e3+e2*e0)*w;
    pedot = 2*(e1*e2+e3*e0)*u + (e2^2+e0^2-e1^2-e3^2)*v + 2*(e2*e3-e1*e0)*w;
    pddot = 2*(e1*e3-e2*e0)*u + 2*(e2*e3+e1*e0)*v + (e3^2+e0^2-e1^2-e2^2)*w;
    
    udot = r*v-q*w + fx/mass;
    vdot = p*w-r*u + fy/mass;
    wdot = q*u-p*v + fz/mass;
    
    %{       
    e0dot = 0.5*(0*e0-p*e1-q*e2-r*e3);
    e1dot = 0.5*(p*e0+0*e1+r*e2-q*e3);
    e2dot = 0.5*(q*e0-r*e1+0*e2+p*e3);
    e3dot = 0.5*(r*e0+q*e1-p*e2+0*e3);
    %}


        lmbda = 1000;
    e_mag = norm([e0 e1 e2 e3]);

    e0dot = 0.5*((lmbda*(1-e_mag^2))*e0-p*e1-q*e2-r*e3);
    e1dot = 0.5*(p*e0+(lmbda*(1-e_mag^2))*e1+r*e2-q*e3);
    e2dot = 0.5*(q*e0-r*e1+(lmbda*(1-e_mag^2))*e2+p*e3);
    e3dot = 0.5*(r*e0+q*e1-p*e2+(lmbda*(1-e_mag^2))*e3);
   
        
    pdot = c1*p*q-c2*q*r + c3*ell+c4*n;
    qdot = c5*p*r-c6*(p^2-r^2) + m/Iy;
    rdot = c7*p*q-c1*q*r + c4*ell+c8*n;
        

sys = [pndot; pedot; pddot; udot; vdot; wdot; e0dot; e1dot; e2dot; e3dot; pdot; qdot; rdot];

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
    
    %xprev = struct(pn,{},pe,{},pd,{},u,{},v,{},w,{},phi,{},theta,{},psi,{});
    load('xprev.mat');
    %pn = class(xprev.a1);
    
    %save('xprev2.mat','pn');

    xlast(1)    = xprev.a1;
    xlast(2)    = xprev.a2;
    xlast(3)    = xprev.a3;
    xlast(4)    = xprev.a4;
    xlast(5)    = xprev.a5;
    xlast(6)    = xprev.a6;
    xlast(7)    = xprev.a7;
    xlast(8)    = xprev.a8;
    xlast(9)    = xprev.a9;
    xlast(10)   = xprev.a10;
    xlast(11)   = xprev.a11;
    xlast(12)   = xprev.a12;

    x8 = xlast(8);
    save('xlast8.mat','x8');
    
    %{
    xprev(1) = M(m_rows, 1);
    xprev(2) = M(m_rows, 2);
    xprev(3) = M(m_rows, 3);
    xprev(4) = M(m_rows, 4);
    xprev(5) = M(m_rows, 5);
    xprev(6) = M(m_rows, 6);
    xprev(7) = M(m_rows, 7);
    xprev(8) = M(m_rows, 8);
    xprev(9) = M(m_rows, 9);
    xprev(10) = M(m_rows, 10);
    xprev(11) = M(m_rows, 11);
    xprev(12) = M(m_rows, 12);
    %}
    
    y(1) = x(1);
    y(2) = x(2);
    y(3) = x(3);
    y(4) = x(4);
    y(5) = x(5);
    y(6) = x(6);

    e0    = x(7);
    e1    = x(8);
    e2    = x(9);
    e3    = x(10);

    vSmall = 10^-5;

     
    if (e0*e1+e2*e3 < vSmall) && (e0^2+e3^2-e1^2-e2^2 < vSmall)
      y(7) = 0;
    else
    y(7) = atan2(2*(e0*e1+e2*e3),e0^2+e3^2-e1^2-e2^2);
    end

    P_exp = 2*(e0*e2-e1*e3);
        
    %y(8) = asin(P_exp);
    
    
    
    if(xlast(11)>=0)
        y(8) = x8 + abs(asin(P_exp)-x8);
      elseif(xlast(11)<0)
        y(8) = x8 - abs(asin(P_exp)-x8);
    end

    
    if((y(8)>2*pi)||(y(8)<-2*pi))
        y8 = abs(y(8))/(2*pi);
      if (y(8)>0)
        y(8) = 2*pi*(y8-floor(y8));
      else
        y(8)=2*pi*(-1*(y8-floor(y8)));
      end
    end
    
    

    
    

    %y(8) = asin(P_exp);

    %y(8) = xprev(8);

    %{
    if (P_exp>-1+vSmall && P_exp<1-vSmall)
      y(8) = asin(P_exp);
    end
    if (P_exp>=1-vSmall && P_exp<=1)
      P_exp = 1+vSmall;
      y(8) = pi/2 + asin(P_exp-1);
    end
    if (P_exp>1+vSmall && P_exp<2-vSmall)
      y(8) = pi/2 + asin(P_exp-1);
    end
    if (P_exp>=2-vSmall && P_exp<=2)
      y(8) = -pi + asin(2-P_exp);
    end
    if (P_exp>2 && P_exp<3-vSmall)
      y(8) = -pi + asin(P_exp-2);
    end
    if (P_exp>=3-vSmall && P_exp<=4-vSmall)
      y(8) = -pi/2 + asin(P_exp-3);
    end
    if (P_exp>-2+vSmall && P_exp<=-1+vSmall)
      y(8) = -pi/2 - asin(abs(P_exp)-1);
    end
    if (P_exp>-3+vSmall && P_exp<=-2+vSmall)
      y(8) = -pi - asin(abs(P_exp)-2);
    end
    if (P_exp>-4+vSmall && P_exp<=-3+vSmall)
      y(8) = pi - asin(abs(P_exp)-3);
    end
    %}

   
    if (e0*e3+e1*e2 < vSmall) && (e0^2+e1^2-e2^2-e3^2 < vSmall)
      y(9) = 0;
    else
    y(9) = atan2(2*(e0*e3+e1*e2),e0^2+e1^2-e2^2-e3^2);
    end
    

    
    

    y(10) = x(11);
    y(11) = x(12);
    y(12) = x(13);

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
