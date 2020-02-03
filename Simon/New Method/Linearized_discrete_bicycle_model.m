function [A,B,x_new] = Linearized_discrete_bicycle_model(x,u,dt)   
% "Linearize Discretize"
A = [1 0 -dt*x(4)*sin(x(3))  dt*cos(x(3))  0;
     0 1 dt*x(5)*cos(x(3))   dt*sin(x(3))  0;
     0 0       1           dt*x(5)       dt*x(4);
     0 0       0            1              0;
     0 0       0            0              1];
%      % U = [uv uk]
B = [0 0;
    0 0; 
    0 0;
    dt*1 0;
    0 dt*1];

x_new = A*x + B*u;

end
    

    