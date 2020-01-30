clc; clear; close all;

sys = ss([0 0; 0 0], [1 0; 0 1],[1 1],[]);
% eig(sys)
sysd = c2d(sys,0.2);


%%%%%%%%%%% Settings %%%%%%%%%%% 
% X = [x1 x2] = [x y]
x(1,:) = [1 1];
N = 20; % Horizon
xf = [16 13];
umax=5;
umin=-5;
Nsim = 10*10;
pauseTime = 0.001;
% obj in the way:
obstacle = [];
% obstacle = [3 0;
%     3.5 0.1;
%     3.5 0.5;
%     3 0.5];
d = zeros(Nsim,2);
% d(3,:) =    [0 2.3];
% d(10,1) =   0;
% d(25,:) =   [-3.7 -2.1];
options = optimoptions(@fmincon,'Algorithm','sqp-legacy','MaxIterations',1500,...
    'OptimalityTolerance',10^(-16),'ConstraintTolerance',10^-8);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
scatter(xf(1),xf(2),50,'g');
hold on 

% patch([obstacle(:,1); obstacle(1,1)], [obstacle(:,2); obstacle(1,2)], 'b', 'FaceAlpha',0.3)
scatter(obstacle(1,1),obstacle(1,2),'k','LineWidth',1.5)
% legend('Goal state','Obstacle')
hold on
xlim([-40 40])
ylim([-40 40])

xcont = linspace(-30,30);
ycont = linspace(-30,30);
[X,Y] = meshgrid(xcont,ycont);
Z =(X-xf(1)).^2 + (Y-xf(2)).^2 + 1./(abs((X-obstacle(1,1))).^2 + (Y-obstacle(1,2)).^2) ;
contour(X,Y,Z,100)
hold on

u = zeros(Nsim,2);

umin=abs(umin);
for k = 1:Nsim   % Arbitrary simulation time.
    %% QUADPROG
%     [Z,fval] = simpleOpti(sysd.A,sysd.B,N,xf,x(k,:),obstacle);
%     u(k,:) = Z(2*N+1:2*N+2);   % Control Horizon
    
    %% NONLINEAR
    [Z,fval] = nonlinearOpti(sysd.A,sysd.B,N,xf,[x(k,:) 0 0],obstacle,umax,umin,options);
    u(k,:) = Z((2*N+1):(2*N+2));
    % Assign input
    x(k+1,:) = sysd.A*x(k,:)'+sysd.B*u(k,:)'+d(k,:)';
    plot(x(k,1),x(k,2),'*')
    xlabel('pos x')
    ylabel('pos y')
    delete(findall(gcf,'type','annotation'))
    annotation('textbox', [0.5, 0.2, 0.1, 0.1], 'String', "fval:" + fval)
    hold on 
    pause(pauseTime)
end

xlabel('x')
ylabel('y')
title("Simulation horizon:" + Nsim + " iterations")
hold on
% plot([boundry(:,1); boundry(1,1)], [boundry(:,2); boundry(1,2)])
% legend('Pos','Iter stamp', 'Obstacle')

