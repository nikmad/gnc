function yf = vtol_dynamics(yi,uu,vtol)

h = 0.01;

yf = rk4(yi,uu,h,vtol);
% yf(12) = 45*pi/180;
end

