function dydt = sixDOF(y,uu)

    pn    = y(1);
    pe    = y(2);
    pd    = y(3);
    u     = y(4);
    v     = y(5);
    w     = y(6);
    phi   = y(7);
    theta = y(8);
    psi   = y(9);
    p     = y(10);
    q     = y(11);
    r     = y(12);
    
%     q     = 45*pi/180;
    
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
   
    % Data for Aerosonde UAV
%     vtol.gravity = 9.81;
% 	vtol.mass = 11.0;
% 	vtol.Jx   = 0.8244;
% 	vtol.Jy   = 1.135;
% 	vtol.Jz   = 1.759;
% 	vtol.Jxz  = 0.1204;
    
    % Gamma parameters 
% 	vtol.Gamma  = Jx*Jz-Jxz^2;
% 	vtol.Gamma1 = (Jxz*(Jx-Jy+Jz))/Gamma;
% 	vtol.Gamma2 = (Jz*(Jz-Jy)+Jxz*Jxz)/Gamma;
% 	vtol.Gamma3 = Jz/Gamma;
% 	vtol.Gamma4 = Jxz/Gamma;
% 	vtol.Gamma5 = (Jz-Jx)/Jy;
% 	vtol.Gamma6 = Jxz/Jy;
% 	vtol.Gamma7 = (Jx*(Jx-Jy)+Jxz*Jxz)/Gamma;
% 	vtol.Gamma8 = Jx/Gamma;
    
%     mass = vtol.mass;
%     Ix = vtol.Jx;
%     Iy = vtol.Jy;
%     Iz = vtol.Jz;
%     Ixz= vtol.Jxz;
    
    mass = 13.5;
    Ix = 0.824400000000000;
    Iy = 0.824400000000000;
    Iz = 0.824400000000000;
    Ixz= 0.824400000000000;

    c0=1.435623440000000; 
    c1=1.435623440000000;    
    c2=0.774654501322436;                                              
    c3=1.225251657913861;
    c4=0.083866003190920;
    c5=0.823436123348018;
    c6=0.106079295154185;
    c7=-0.168263120585437;
    c8=0.574245290951783;

    pndot = u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)) ...
        + w*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
    pedot = u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)) ...
        + w*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
    pddot = -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta);

    udot = r*v-q*w + fx/mass;
    vdot = p*w-r*u + fy/mass;
    wdot = q*u-p*v + fz/mass;

	vSmall = 0.005;

%____________________________________________________________
	% nkm - "Around 2.8648 deg (in rad 0.05) below and above 90 deg
	% has to be discarded for sec() and tan() functions because
	% in this zone, increase in sec(theta) and tan(theta) for a
	% small rise in theta is huge. So we discard the zone (90-2.86)
	% to (90+2.86) i.e., 87.14 to 92.86 deg".

	sensitive90zone = 0.05; 
	
%____________________________________________________________

theta1 = mod(abs(theta),2*pi);
theta2 = floor(abs(theta)/(2*pi))*(2*pi);

if     ((floor(theta1/(pi/2))==0) &&   (pi/2-theta1 < sensitive90zone))
	theta = sign(theta)*(pi/2 - sensitive90zone + theta2);

elseif ((floor(theta1/(pi/2))==1) &&   (theta1-pi/2 < sensitive90zone))
	theta = sign(theta)*(pi/2 + sensitive90zone + theta2);

elseif ((floor(theta1/(pi/2))==2) && (3*pi/2-theta1 < sensitive90zone))
	theta = sign(theta)*(3*pi/2 - sensitive90zone + theta2);

elseif ((floor(theta1/(pi/2))==3) && (theta1-3*pi/2 < sensitive90zone))
	theta = sign(theta)*(3*pi/2 + sensitive90zone + theta2);
end

    phidot = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
	thetadot = q*cos(phi)-r*sin(phi);
%     thetadot  = 0;
	psidot = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
        
    pdot = c1*p*q-c2*q*r + c3*ell+c4*n;
    qdot = c5*p*r-c6*(p^2-r^2) + m/Iy;
%     qdot = 45*pi/180;
    rdot = c7*p*q-c1*q*r + c4*ell+c8*n;
        
dydt = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];
end