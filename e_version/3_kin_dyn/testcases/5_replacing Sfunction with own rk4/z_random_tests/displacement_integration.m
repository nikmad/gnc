function s = displacement_integration()

f = 0.5023; % force in N
t = 10;     %total time in s
m = 13.5;   %kg
del_t = 0.1; % time-step
u = 0; % initial velocity
s = 0; %displacement

for i = 0 : del_t : t
    s = s + u*del_t + 0.5 * f/m * del_t^2;
    u = u + f/m * del_t;
end

end