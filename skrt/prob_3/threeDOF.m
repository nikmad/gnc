function dydt = threeDOF(y,uu,scraft)

hx      = y(1);
hy      = y(2);
hz      = y(3);
wx      = y(4);
wy      = y(5);
wz      = y(6);
phi     = y(7);
theta   = y(8);
psi     = y(9);

tauX = uu(1);
tauY = uu(2);
tauZ = uu(3);

hxdot = tauX;
hydot = tauY;
hzdot = tauZ;

wxdot = (1/scraft.Jx) * ((scraft.Jy-scraft.Jz)*wy*wz - hxdot + hy*wz - hz*wy - 3 * scraft.n^2 * (scraft.Jy-scraft.Jz) * sin(phi) * cos(phi) * (cos(theta))^2);
wydot = (1/scraft.Jy) * ((scraft.Jz-scraft.Jx)*wx*wz - hydot + hz*wx - hx*wz + 3 * scraft.n^2 * (scraft.Jz-scraft.Jx) * cos(phi) * sin(theta) * cos(theta));
wzdot = (1/scraft.Jz) * ((scraft.Jx-scraft.Jy)*wx*wy - hzdot + hx*wy - hy*wx + 3 * scraft.n^2 * (scraft.Jx-scraft.Jy) * sin(phi) * sin(theta) * cos(theta));

phidot = wx + (wy*sin(phi)*sin(theta) + wz*cos(phi)*sin(theta))/cos(theta) + scraft.n * sin(psi) * cos(theta);
thetadot = wy*cos(phi) - wz*sin(phi) + scraft.n * cos(psi); 
psidot = (wy*sin(phi) + wz*cos(phi))/cos(theta) + scraft.n * sin(psi) * sin(theta) / cos(theta);

dydt = [hxdot; hydot; hzdot; wxdot; wydot; wzdot; phidot; thetadot; psidot];

end

