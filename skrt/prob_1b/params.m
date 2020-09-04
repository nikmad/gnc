
% Kp = 1000 and Kd = 1700 gives all 4-stable poles with damping factor or atleast 0.3 
Kp = 1000;
Kd = 1700;

% structural parameters
J   = 2000; % kg-m^2. MoI
swz = 1;    % rad/s. structural freq wz
swp = 1.3;  % rad/s. structural freq wp
sdr = 0.002; % structural damping ratio

% Numerator coeffs of CL-TF
b0 = Kd/swz^2; % S^3 coeff
b1 = Kp/swz^2 + 2*sdr*Kd/swz; % S^2 coeff
b2 = 2*sdr*Kp/swz + Kd; % S^1 coeff
b3 = Kp; % S^0 coeff

% Denominator coeffs of CL-TF
a0 = J/swp^2; % S^4 coeff
a1 = 2*sdr*J/swp + Kd/swz^2; % S^3 coeff
a2 = J + 2*sdr*Kd/swz + Kp/swz^2; % S^2 coeff
a3 = Kd + 2*sdr*Kp/swz; % S^1 coeff
a4 = Kp; % S^0 coeff



