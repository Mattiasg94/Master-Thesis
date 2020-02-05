function [A,B] = Linearized_discrete_DD_model(xr,ur,dt)
% "Discretize Linearize"
A = [ 1, 0, -dt*ur(1)*sin(xr(3));
      0, 1,  dt*ur(1)*cos(xr(3));
      0, 0,    1];

B = [ dt*cos(xr(3)) 0;
      dt*sin(xr(3)) 0;
      0     dt];

% A = [ 1, ur(2)*dt, 0;
%       ur(2)*dt, 1,  ur(1)*dt;
%       0, 0,    1];
% 
% B = [ -dt 0;
%       0 0;
%       0 -dt];
end


