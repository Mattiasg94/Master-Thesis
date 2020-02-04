function [A,B] = Linearized_discrete_DD_model(x,u,dt)
% "Discretize Linearize"
% A = [ 1, 0, -dt*u(1)*sin(x(3));
%       0, 1,  dt*u(1)*cos(x(3));
%       0, 0,    1];

A = [ 1, 0, 0;
      0, 1,  0;
      0, 0,    1];
B = [ dt*cos(x(3)) 0;
      dt*sin(x(3)) 0;
      0     dt*1];
end


