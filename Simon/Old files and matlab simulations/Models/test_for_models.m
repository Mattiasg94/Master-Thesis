clc;clear;close all;

%% Test differential drive
Ts = 0.1;
x = [2 2 pi/2]';
% u = [3 3]';
N = 10;
u = [];
x_traj = x;
for i = 1:N
    u = [u [0.1*i; 2*i]];
    [A,B,x] = Linearized_discrete_DD_model(x_traj(:,i),u(:,i),Ts);   
    x_traj(:,i+1) = x;
    plot(x_traj(1,:),x_traj(2,:))
    xlim([-10 10])
    ylim([-10 10])
    hold on
    pause(0.5)
end

%% Test bicycle model
clc;clear;close all;
Ts = 0.1;
x = [2 2 pi/2 0 0]';
% u = [3 3]';
N = 10;
u = [];
x_traj = x;
for i = 1:N
    u = [u [i; i]];
    [A,B,x] = Linearized_discrete_bicycle_model(x_traj(:,i),u(:,i),Ts);   
    x_traj(:,i+1) = x;
    plot(x_traj(1,:),x_traj(2,:))
    xlim([-10 10])
    ylim([-10 10])
    hold on
    pause(0.5)
end


%% Tests for linearization
% clc
% clear
% u = sym('u',[2 1]);
% x = sym('x',[3 1]);
% 
% Matrix = [ u(1)*cos(x(3)) %+ cos(x(3));
%       u(1)*sin(x(3)) %+ sin(x(3));
%             u(2)       ]; 
% v = [x;u]
% Jac = jacobian(Matrix,v)
% 
% A = [0 0 Jac(1,3);
%      0 0 Jac(2,3);
%      0 0 Jac(3,3)]
%  B = [0 Jac(1,4);
%       0 Jac(2,4);
%       0    1    ]
%  
%  
