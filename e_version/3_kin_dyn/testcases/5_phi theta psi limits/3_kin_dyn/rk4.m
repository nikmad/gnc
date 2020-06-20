function [ sol, t ] = rk4( A,B,X0,dt,ti,tf,order )
% Inputs
% A              : state matix of the diffrential equations  (nxn)
% B              : B matix of the diffrential equations (nx1)
% X0            : initial condition matix (nx1)
% dt             : time step
% t_initial    : initial time
% t_final      : final time
% order       : n
% Outputs
% sol              : the solution of diffrential equation [x, dx/dt, d2x/dt2, ... , d(order)x/dt(order) ; t]
% t                  : solution time

% Function body

n=(tf-ti)/dt;
X(1:order,1)=X0;
t(1)=ti;
for m=2:ceil(n)+1
    k1=(A*X(1:order,m-1)+B)*dt;
    k2=(A*(X(1:order,m-1)+k1/2)+B)*dt;
    k3=(A*(X(1:order,m-1)+k2/2)+B)*dt;
    k4=(A*(X(1:order,m-1)+k3)+B)*dt;
    X(1:order,m)=X(1:order,m-1)+(k1+2*k2+2*k3+k4)*1/6;
    t(m)=t(m-1)+dt;
end
sol=[X];
end