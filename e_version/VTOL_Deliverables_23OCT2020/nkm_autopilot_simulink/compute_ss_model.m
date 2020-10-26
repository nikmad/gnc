function [A_lon, B_lon, A_lat, B_lat] = compute_ss_model(filename, x_trim, u_trim)
% x_trim is the trimmed state,
% u_trim is the trimmed input

[A,B,C,D]=linmod(filename,x_trim,u_trim);

% A, B, C, D represent the following state-space
% X_dot = Ax + Bu
% Y     = Cx + Du
% We are only interested in X_dot equation because we are looking at the
% equations of derivatives of the 12 states

% A: is 12x12 matrix that is formed by concatenating the first matrices on the RHS of
% equations 5.43 and 5.50. Since the first matrix on RHS of 5.43 is 5x6 and
% first matrix on RHS of 5.50 is 5x6 we get by concatenating the two
% matrices, a 12x10 matrix. Moreover note that the last column of both the
% matrices is fully zeroes. Hence we have indeed a 12x9 matrix. Hence, 3
% columns of A will be fully zeroes. You can check this by printing A to
% the command window.

% B: is a 3x4 matrix. The number 4 represents 4 control inputs. 
% C: is 3x12
% D: is 3x4

E1 = [...
0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;... % Enables selection of 1st row of 5x5 Matrix on RHS of Eq 5.43 to construct v_bar_dot equation 
0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;... % Enables selection of 2nd row of 5x5 Matrix on RHS of Eq 5.43 to construct p_bar_dot equation 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;... % Enables selection of 3rd row of 5x5 Matrix on RHS of Eq 5.43 to construct r_bar_dot equation 
0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;... % Enables selection of 4th row of 5x5 Matrix on RHS of Eq 5.43 to construct phi_bar_dot equation 
0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0;... % Enables selection of 5th row of 5x5 Matrix on RHS of Eq 5.43 to construct psi_bar_dot equation 
];

E2 = [...
0, 1, 0, 0;... % Enables selection of delta_a coefficients in Eq 5.43
0, 0, 1, 0;... % Enables selection of delta_r coefficients in Eq 5.43
];

A_lat = E1 * A * E1';
B_lat = E1 * B * E2';

E3 = [...
0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;... % Enables selection of 1st row of 5x5 Matrix on RHS of Eq 5.50 to construct u_bar_dot equation 
0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;... % Enables selection of 2nd row of 5x5 Matrix on RHS of Eq 5.50 to construct w_bar_dot equation 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;... % Enables selection of 3rd row of 5x5 Matrix on RHS of Eq 5.50 to construct q_bar_dot equation 
0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;... % Enables selection of 4th row of 5x5 Matrix on RHS of Eq 5.50 to construct theta_bar_dot equation 
0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 0;... % Enables selection of 5th row of 5x5 Matrix on RHS of Eq 5.50 to construct alt_bar_dot equation 
];

E4 = [...
1, 0, 0, 0;... % Enables selection of delta_e coefficients in Eq 5.50
0, 0, 0, 1;... % Enables selection of delta_t coefficients in Eq 5.50
];

A_lon = E3 * A * E3';
B_lon = E3 * B * E4';

end
  