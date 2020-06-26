function [x_trim,u_trim,y_trim,dx_trim] = compute_trim(filename, Va, gamma, R)

% set initial conditions 
x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
%x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

% specify which states to hold equal to the initial conditions
ix = [];

% specify initial inputs 
u0 = [...
    0;... % 1 - delta_e
    0;... % 2 - delta_a
    0;... % 3 - delta_r
    1;... % 4 - delta_t
    ];	

% specify which inputs to hold constant
iu = [];

% define constant outputs
y0 = [...
    Va;...       % 1 - Va
    0;...    % 2 - alpha
    0;...        % 3 - beta
    ];

% specify which outputs to hold constant
iy = [1,3];

% define constant derivatives
dx0 = [0; 0; Va*sin(gamma); 0; 0; 0; 0; 0; Va/R*cos(gamma); 0; 0; 0];
%dx0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

if R~=Inf, dx0(9) = Va/R; end % 9 - psidot   

% specify which derivaties to hold constant in trim algorithm
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

x0
u0
y0
ix
iu
iy
dx0
idx
% compute trim conditions
[x_trim,u_trim,y_trim,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);
x_trim
u_trim
y_trim
dx_trim

% check to make sure that the linearization worked (should be small)
Linearization_coeff = norm(dx_trim(3:end)-dx0(3:end))

end


