function [A,B,x_new] = Linearized_discrete_DD_model(x,u,dt)
x = x(:);
u = u(:);
% "Discretize Linearize"
A = [ 1, 0, -dt*u(1)*sin(x(3));
      0, 1,  dt*u(1)*cos(x(3));
      0, 0,    1];
      
B = [ dt*cos(x(3)) 0;
      dt*sin(x(3)) 0;
      0     dt*1];
% R = 0.5;
% L = 1;
% 
% A = [ 1, 0, -dt*R/2*(u(1)+u(2))*sin(x(3));
%       0, 1,  dt*R/2*(u(1)+u(2))*cos(x(3));
%       0, 0,    1];
%       
% B = [ dt*R/2*cos(x(3))  dt*R/2*cos(x(3));
%       dt*R/2*sin(x(3))  dt*R/2*sin(x(3));
%       dt*R/L           -dt*R/L];  
  
x_new = A*x + B*u;

end


