function yf = vtol_dynamics(yi,uu,t,vtol)

h = 0.01;

if t<0.2
    yi(1) = vtol.pn0;
	yi(2) = vtol.pe0;
	yi(3) = vtol.pd0;
	yi(4) = vtol.u0;
	yi(5) = vtol.v0;
	yi(6) = vtol.w0;
	yi(7) = vtol.phi0;  
	yi(8) = vtol.theta0; 
	yi(9) = vtol.psi0;
	yi(10) = vtol.p0;
	yi(11) = vtol.q0;
%     yi(11) = 45*pi/180;
	yi(12) = vtol.r0;  
end
    
yf = rk4(yi,uu,h,vtol);
% yf(12) = 45*pi/180;
end

