close all;clear;clc;
%% You can use any command in MPT in this assignment

%% Question 1
% write the code to plot X0. In the report, give Pf,  the X0, and your
% motivation for choosing this Pf.
clc, clearvars
A = [1.2, 1; 0, 1];
B = [0; 1];
u_lim = 1;
x_lim = 15;
Q = eye(2);
R = 100;
N = 3;

[~, Pf, ~] = dlqr(A, B, Q, R);

Xf = Polyhedron('V', [0,0]);
model = LTISystem('A', A, 'B', B);
model.x.max = [x_lim; x_lim];
model.x.min = -[x_lim; x_lim];
model.u.max = u_lim;
model.u.min = -u_lim;
ReachableSet = Xf;
X = Polyhedron([-1, 0; 1, 0; 0, -1; 0, 1], [x_lim; x_lim; x_lim; x_lim]);
U = Polyhedron([-1; 1], [u_lim; u_lim]);
for k = 1:N
    ReachableSet = intersect(model.reachableSet('X', ReachableSet, 'U', U, 'direction', 'backwards'), X);
end
X0 = ReachableSet;
figure(1)
plot(X0)
xlabel('x_1', 'FontSize', 14)
ylabel('x_2', 'FontSize', 14)
title('X_0', 'FontSize', 16)

%% Question 2
% write the code to plot the requested figures. Provide the figures in your
% report and explain your observation about the error between the state and
% state prediction as N increases.
clearvars, clc
A = [1.2, 1; 0, 1];
B = [0; 1];
u_lim = 1;
x_lim = 15;
Q = eye(2);
R = 100;
x0 = [4; -2.6];

Xf = Polyhedron('V', [0,0]);

model = LTISystem('A', A, 'B', B);
model.x.max = [x_lim; x_lim];
model.x.min = -[x_lim; x_lim];
model.u.max = u_lim;
model.u.min = -u_lim;
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);

model.x.with('terminalSet');
model.x.terminalSet = Xf;
model.x.with('terminalPenalty');
model.x.terminalPenalty = model.LQRPenalty;

N = 10;
ctrl10 = MPCController(model, N);
[u, feasible, openloop] = ctrl10.evaluate(x0);
Xpred = openloop.X;
loop = ClosedLoop(ctrl10, model);
data = loop.simulate(x0, N);
Xsim = data.X;

figure(1)
plot(0:N, Xpred(1, :))
hold on
plot(0:N, Xpred(2, :))
plot(0:N, Xsim(1, :))
plot(0:N, Xsim(2, :))
title("Predicted x Vs. actual x for  N = " + num2str(N))
legend('Predicted x_1','Predicted x_2', 'Actual x_1', 'Acutal x_2')
xlabel('Time')
hold off


N = 15;
ctrl15 = MPCController(model, N);
[u, feasible, openloop] = ctrl15.evaluate(x0);
Xpred = openloop.X;
loop = ClosedLoop(ctrl15, model);
data = loop.simulate(x0, N);
Xsim = data.X;

figure(2)
plot(0:N, Xpred(1, :))
hold on
plot(0:N, Xpred(2, :))
plot(0:N, Xsim(1, :))
plot(0:N, Xsim(2, :))
title("Predicted x Vs. actual x for  N = " + num2str(N))
legend('Predicted x_1','Predicted x_2', 'Actual x_1', 'Actual x_2')
xlabel('Time')
hold off


N = 20;
ctrl20 = MPCController(model, N);
[u, feasible, openloop] = ctrl20.evaluate(x0);
Xpred = openloop.X;
Xsim = zeros(size(Xpred));
Xsim(:, 1) = x0;
loop = ClosedLoop(ctrl20, model);
data = loop.simulate(x0, N);
Xsim = data.X;

figure(3)
plot(0:N, Xpred(1, :))
hold on
plot(0:N, Xpred(2, :))
plot(0:N, Xsim(1, :))
plot(0:N, Xsim(2, :))
title("Predicted x Vs. actual x for  N = " + num2str(N))
legend('Predicted x_1','Predicted x_2', 'Actual x_1', 'Actual x_2')
xlabel('Time')
hold off




%% Question 3
% no code is needed. Answer in the report

%% Question 4
% write a code that calculates the figures and costs. Provide the figures
% and costs in the report. for costs, provide a table in the report that
% provides all costs for all different methods in the question (4 methods,
% each with three different costs as defined in A7 assignment). If you what
% to use some functions in the code, you can write them in different matlab
% files and submit them with the rest of your files
clc,clear all

% C2 case 1
clc, clearvars
C = 9.2e3; % kJ/C
R = 50; % C*s/kJ
dt = 3600; %s
N = 24;
TLb = 21; TUb = 26;
A = 1 - dt/(R*C); B = dt/C;
rho = 1000; kappa = 2;
T0 = 22;
data = load('A7_data');
Pd = data.Pd;
T_oa = data.T_oa;
d = Pd(1:N)*dt/C + T_oa(1:N)*dt/(R*C);



numberOfDays = 7;
Nsim = 24*numberOfDays;
T = zeros(Nsim + 1, 1);
T(1) = T0;
TNoInput = zeros(Nsim + 1, 1);
TNoInput(1) = T0;
u = zeros(Nsim , 1);

dsim = repmat(d, [numberOfDays, 1]); % repeats d so it is the same all days
ToaV=T_oa(1:N)*dt/(R*C)
ToaV_sim = repmat(ToaV, [numberOfDays, 1]);
for k = 2:Nsim+1
    dCurrent = circshift(d,-(k-2));
    Z = optimizer_23(A, B, N, TUb, TLb, T(k-1), dCurrent, rho, kappa);
    u(k-1) = Z(1);
    T(k) = A*T(k-1) + B*u(k-1) + dsim(k-1);
    
    TNoInput(k) = A*TNoInput(k-1)+ToaV_sim(k-1); % + dsim(k-1);
end
plot(0:Nsim,T)
xlim([numberOfDays*24 - 24, numberOfDays*24])
ylim([21, 27])
set(gca, 'FontSize', 14)
xlabel('Time (h)')
ylabel('Temperature (C)')
title('C2 case (i)')

us = u(end - 23:end);
Ts = T(end - 23:end);
epss = [(Ts >= TUb).*(Ts - TUb); (Ts <= TLb).*(TLb - Ts)];
[Ju, Jp, Jeps] = costs_23(us, epss, dt)
figure(3)
plot(TNoInput)
figure(4)
plot(ToaV_sim)
%% C2 case 2
clc, clearvars
C = 9.2e3; % kJ/C
R = 50; % C*s/kJ
dt = 3600; %s
N = 24;
TLb = 21; TUb = 26;
A = 1 - dt/(R*C); B = dt/C;
rho = 1000; kappa = 2;
T0 = 22;
data = load('A7_data');
Pd = data.Pd;
T_oa = data.T_oa;
d = Pd(1:N)*dt/C + T_oa(1:N)*dt/(R*C);


numberOfDays = 7;
Nsim = 24*numberOfDays;
T = zeros(Nsim + 1, 1);
T(1) = T0;
TNoInput = zeros(Nsim + 1, 1);
TNoInput(1) = T0;
u = zeros(Nsim , 1);

dsim = repmat(d, [numberOfDays, 1]);

for k = 2:Nsim+1
    dCurrent = circshift(d,-(k-2));
    dPrediction = ones(size(d))*dCurrent(1);
    Z = optimizer_23(A, B, N, TUb, TLb, T(k-1), dPrediction, rho, kappa);
    u(k-1) = Z(1);
    T(k) = A*T(k-1) + B*u(k-1) + dsim(k-1);
    
    TNoInput(k) = A*TNoInput(k-1) + dsim(k-1);
end
plot(0:Nsim,T)

xlim([numberOfDays*24 - 24, numberOfDays*24])
ylim([24, 27])
set(gca, 'FontSize', 14)
xlabel('Time (h)')
ylabel('Temperature (C)')
title('C2 case (ii)')

us = u(end - 23:end);
Ts = T(end - 23:end);
epss = [(Ts >= TUb).*(Ts - TUb); (Ts <= TLb).*(TLb - Ts)];
[Ju, Jp, Jeps] = costs_23(us, epss, dt)

%% C2 case 3
clc, clearvars
C = 9.2e3; % kJ/C
R = 50; % C*s/kJ
dt = 3600; %s
N = 24;
TLb = 21; TUb = 26;
A = 1 - dt/(R*C); B = dt/C;
rho = 1000; kappa = 2;
T0 = 22;
data = load('A7_data');
Pd = data.Pd;
T_oa = data.T_oa;
d = Pd(1:N)*dt/C + T_oa(1:N)*dt/(R*C);

numberOfDays = 7;
Nsim = 24*numberOfDays;
T = zeros(Nsim + 1, 1);
T(1) = T0;
TNoInput = zeros(Nsim + 1, 1);
TNoInput(1) = T0;
u = zeros(Nsim , 1);

dsim = repmat(d, [numberOfDays, 1]);
dEstimate = 0;

for k = 2:Nsim+1
    Z = optimizer_23(A, B, N, TUb, TLb, T(k-1), dEstimate*ones(size(d)), rho, kappa);
    u(k-1) = Z(1);
    T(k) = A*T(k-1) + B*u(k-1) + dsim(k-1);
    dEstimate = T(k) - A*T(k-1) - B*u(k-1);
    
    TNoInput(k) = A*TNoInput(k-1) + dsim(k-1);
end
plot(0:Nsim,T)

xlim([numberOfDays*24 - 24, numberOfDays*24])
ylim([24, 26.5])
set(gca, 'FontSize', 14)
xlabel('Time (h)')
ylabel('Temperature (C)')
title('C2 case (iii)')


us = u(end - 23:end);
Ts = T(end - 23:end);
epss = [(Ts >= TUb).*(Ts - TUb); (Ts <= TLb).*(TLb - Ts)];
[Ju, Jp, Jeps] = costs_23(us, epss, dt)
%% C1 case 3

clc, clearvars

C = 9.2e3; % kJ/C
R = 50; % C*s/kJ
K = 400;
dt = 60; %s
TLb = 21; TUb = 26;
A = 1 - dt/(R*C); B = dt/C;
rho = 1000; kappa = 2;
T0 = 22;
data = load('A7_data');
Pd = data.Pd;
T_oa = data.T_oa;
d = Pd*dt/C + T_oa*dt/(R*C);
numberOfDays = 7;
Nsim = 60*24*numberOfDays;

T = zeros(Nsim + 1, 1);
T(1) = T0;
TNoInput = zeros(Nsim + 1, 1);
TNoInput(1) = T0;
u = zeros(Nsim , 1);

for k = 2:Nsim+1 
    if T(k-1) >= TUb
        u(k-1) = K*(TUb - T(k-1));
    elseif T(k-1) <= TLb
        u(k-1) = K*(TLb - T(k-1));
    else
        u(k-1) = 0;
    end
    
    % Needed to find out what d was this particular minute.
    % Considered d to be constant each hour.
    % Could have also just made d a number_of_days*24*60 vector, but
    % decided to to these calculations to find the current hour.
    minuteThatDay = mod((k-1), 60*24);
    hourThatDay = fix(minuteThatDay/ 60) + 1; 

    T(k) = A*T(k-1) + B*u(k-1) + d(hourThatDay);
    TNoInput(k) = A*TNoInput(k-1) + d(hourThatDay);
end
plot((0:Nsim)/60,T)
xlim([numberOfDays*24 - 24, numberOfDays*24])
ylim([24, 26.5])
set(gca, 'FontSize', 14)
xlabel('Time (h)')
ylabel('Temperature (C)')
title('C1 case (iii)')

us = u(end - 24*60 + 1:end);
Ts = T(end - 24*60 + 1:end);
epss = [(Ts >= TUb).*(Ts - TUb); (Ts <= TLb).*(TLb - Ts)];
[Ju, Jp, Jeps] = costs_23(us, epss, dt)





