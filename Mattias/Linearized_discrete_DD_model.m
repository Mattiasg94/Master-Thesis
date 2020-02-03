function [A,B] = Linearized_discrete_DD_model(x,u,dt)
% "Discretize Linearize"
A = [ 1, 0, -dt*u(1)*sin(x(3));
      0, 1,  dt*u(1)*cos(x(3));
      0, 0,    1];
      
B = [ cos(x(3)) 0;
      sin(x(3)) 0;
      0     dt*1];
end


