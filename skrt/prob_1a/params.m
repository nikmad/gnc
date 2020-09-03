
J = 2000;   % Moment of Inertia in kg-m^2
wn = 3;     % natural freq in rad/s
zeta = 0.7; % damping ratio

Kp = J * wn^2;
Kd = 2 * zeta * sqrt(Kp * J);

