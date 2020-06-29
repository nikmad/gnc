function dydt = sixDOF(y,uu)

    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);

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
    
   
    % Data for Aerosonde UAV
    gravity = 9.81;
	mass = 11.0;
	Jx   = 0.8244;
	Jy   = 1.135;
	Jz   = 1.759;
	Jxz  = 0.1204;
    
    % Gamma parameters 
	Gamma  = Jx*Jz-Jxz^2;
	Gamma1 = (Jxz*(Jx-Jy+Jz))/Gamma;
	Gamma2 = (Jz*(Jz-Jy)+Jxz*Jxz)/Gamma;
	Gamma3 = Jz/Gamma;
	Gamma4 = Jxz/Gamma;
	Gamma5 = (Jz-Jx)/Jy;
	Gamma6 = Jxz/Jy;
	Gamma7 = (Jx*(Jx-Jy)+Jxz*Jxz)/Gamma;
	Gamma8 = Jx/Gamma;
    
    mass = mass;
    Ix = Jx;
    Iy = Jy;
    Iz = Jz;
    Ixz= Jxz;

    c0=Gamma; 
    c1=Gamma1;    
    c2=Gamma2;                                              
    c3=Gamma3;
    c4=Gamma4;
    c5=Gamma5;
    c6=Gamma6;
    c7=Gamma7;
    c8=Gamma8;

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
	psidot = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
        
    pdot = c1*p*q-c2*q*r + c3*ell+c4*n;
    qdot = c5*p*r-c6*(p^2-r^2) + m/Iy;
    %qdot = 0;
    rdot = c7*p*q-c1*q*r + c4*ell+c8*n;
        
dydt = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];
end