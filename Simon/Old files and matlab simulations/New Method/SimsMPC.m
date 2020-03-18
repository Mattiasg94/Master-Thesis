clc; clear; close all;

%%%%%%%%%%% Settings %%%%%%%%%%%
% X = [x1 x2] = [x y]
x0 = [4 1 pi/4];    % DD
num_states = length(x0);
dt = 0.1;
N = 1;         % Horizon
xf = [10 10 pi/4];
umax=[5 5];
umin=[-3 -5];  % bin handles (-) sign for umin
Nsim = 60; %*10;   % Simulation steps
pauseTime = 0;
obstacle = [3 0];

% Disturbance
d = zeros(Nsim,num_states);

% Algorithm options:
options = optimoptions(@fmincon,'Algorithm','sqp','MaxIterations',5000,...
    'OptimalityTolerance',10^(-16),'ConstraintTolerance',10^-8);% ,'Display','off');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
reference = [linspace(x0(1),xf(1),Nsim); linspace(x0(2),xf(2),Nsim); linspace(x0(3),xf(3),Nsim)];
u = zeros(Nsim,2);
u0 = [0.1 0.1];
u(1,:) = u0;
x(1,:) = x0;

figure(1)
scatter(xf(1),xf(2),50,'g');
hold on
xlim([0 20])
ylim([0 20])
xcont = linspace(-30,30);
ycont = linspace(-30,30);
[X,Y] = meshgrid(xcont,ycont);
Z =(X-xf(1)).^2 + (Y-xf(2)).^2;
contour(X,Y,Z,300)
hold on
xlabel('pos x')
ylabel('pos y')
num_input = length(u0);

for i = 1:num_states*N + num_input*N % Min+Max vals
    if i >= num_states*N+1 && i <= num_states*N+num_input*N
        if mod(i,2)==0
            ub(i) = umax(1);
        elseif mod(i,2)==1
            ub(i) = umax(2);
        end
    end
    if i >= num_states*N+1 && i >= num_input*N+1
        if mod(i,2)==0
            lb(i) = umin(1);
        elseif mod(i,2)==1
            lb(i) = umin(2);
        end
    end
end

for k = 1:Nsim
    %% QUADPROG%%%%%%%%%%%%
    %     [Z,fval] = simpleOpti(sysd.A,sysd.B,N,xf,x(k,:),obstacle);
    %     u(k,:) = Z(2*N+1:2*N+2);   % Control Horizon
    %%%%%%%%%%%%%%%%%%%%%%%
    %% NONLINEAR %%%%%%%%%%
    % [x(k,:) 0 0]
    Z0 = [];
    for i = 1:N
        Z0 = [Z0; x(k,:)'];
    end
    for i = 1:N
        Z0 = [Z0; u(k,:)'];
    end
%     [A,B,~] = Linearized_discrete_DD_model(xf,[0; 0],dt);
%     [A,B,~] = Linearized_discrete_DD_model(reference(:,k),[0; 0],dt);
    [A,B,~] = Linearized_discrete_DD_model(Z0(1:num_states),Z0(N*num_states+1:N*num_states+num_input),dt);
%     disp('---------------')
%     disp(rank(ctrb(A,B)))
%     disp('---------------')
%     pause(0.5)
    [Z,fval(k)] = nonlinearOpti(A,B,N,xf,Z0,obstacle,ub,lb,options);
    u(k,:) = Z((num_states*N+1):(num_states*N+2));
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Assign input
    x(k+1,:) = A*x(k,:)'+B*u(k,:)'+d(k,:)';
    plot(x(k,1),x(k,2),'*')
    delete(findall(gcf,'type','annotation'))
    annotation('textbox', [0.5, 0.2, 0.1, 0.1], 'String', "fval:" + fval(k))
    title("Current iteration:" + k + ". Horizon N=" + N +".")
    pause(pauseTime)
    
end

figure(2)
plot(u(:,1))
% hold on
% plot(fval)
legend("u_1 (v)")
xlabel("Simulation step " + dt + " sec")
ylabel("Input value, v")

figure(3)
plot(u(:,2))
% hold on
% plot(fval)
legend("u_2 (\omega)")
xlabel("Simulation step " + dt + " sec")
ylabel("Input value, \omega")

