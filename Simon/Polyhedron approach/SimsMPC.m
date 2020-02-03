clc; clear; close all;

%%%%%%%%%%% Settings %%%%%%%%%%% 
x0 = [1 2];
dt = 0.2;
N = 10;         % Horizon
xf = [10 0];
umax=1;
umin=1;
Nsim = 10*10;   % Simulation steps
pauseTime = 0.001;
obstacle = [3 0];

% Disturbance
d = zeros(Nsim,2);

% Algorithm options:
options = optimoptions(@fmincon,'Algorithm','sqp','MaxIterations',5000,...
    'OptimalityTolerance',10^(-16),'ConstraintTolerance',10^-8);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sys = ss([0 0; 0 0], [1 0; 0 1],[1 1],[]);
sysd = c2d(sys,dt);
u = zeros(Nsim,2);
umin=abs(umin);
x(1,:) = x0;

figure(1)
scatter(xf(1),xf(2),50,'g');
hold on 
patch([obstacle(1)-0.5; obstacle(1)+0.5; obstacle(1)+0.5; obstacle(1)-0.5], [obstacle(2)+0.5; obstacle(2)+0.5;obstacle(2)-0.5; obstacle(2)-0.5], 'b', 'FaceAlpha',0.3)
hold on
xlim([0 30])
ylim([-10 10])
xcont = linspace(-30,30);
ycont = linspace(-30,30);
[X,Y] = meshgrid(xcont,ycont);
Z =(X-xf(1)).^2 + (Y-xf(2)).^2;
% + 1./(abs((X-obstacle(1,1))).^2 + (Y-obstacle(1,2)).^2)
contour(X,Y,Z,300)
hold on
xlabel('pos x')
ylabel('pos y')
Z0 = zeros(2*N+2*N+8*N,1); % 2*N*N


for k = 1:Nsim 
    %%%%%%%%%% QUADPROG%%%%%%%%%%%%
%     [Z,fval] = simpleOpti(sysd.A,sysd.B,N,xf,x(k,:),obstacle);
%     u(k,:) = Z(2*N+1:2*N+2);   % Control Horizon
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%% NONLINEAR %%%%%%%%%%
    [Z,fval] = nonlinearOpti(sysd.A,sysd.B,N,xf,Z0,obstacle,umax,umin,options);
    Z0 = Z;
    u(k,:) = Z((2*N+1):(2*N+2));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Assign input
    x(k+1,:) = sysd.A*x(k,:)'+sysd.B*u(k,:)'+d(k,:)';
    
    plot(x(k,1),x(k,2),'*')
    delete(findall(gcf,'type','annotation'))
    annotation('textbox', [0.5, 0.2, 0.1, 0.1], 'String', "fval:" + fval)
    title("Current iteration:" + k + ". Horizon N=" + N +".")
    
    pause(pauseTime)

    
end



