function yi = init_states(t, scraft)

    yi(1) = scraft.hx0;
	yi(2) = scraft.hy0;
	yi(3) = scraft.hz0;
	yi(4) = scraft.wx0;
	yi(5) = scraft.wy0;
	yi(6) = scraft.wz0;
	yi(7) = scraft.phi0;  
	yi(8) = scraft.theta0; 
	yi(9) = scraft.psi0;
end
