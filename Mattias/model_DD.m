clc
clear all
close all
format short

N=10;
Nsim=30;
dt=0.5;
num_states=6;
u = zeros(Nsim , 2);
x = zeros(Nsim + 1, 3);
u_tilde = zeros(Nsim , 2);
x_tilde = zeros(Nsim + 1, 3);
ur=[0.1;0];
xr=[10;5;0];
x0=[0;4;0];
u0=[0;1];
x(1,:) = x0;
lb=[-10  -10 -2*pi -10 -10 -2*pi -1 -1];
ub=[10 10 2*pi 10 10 2*pi 1 1];

lb1=[];lb2=[];lb3=[];
ub1=[];ub2=[];ub3=[];
for i=1:N
    lb1=[lb1,lb(1:3)];
    lb2=[lb2,lb(4:6)];
    lb3=[lb3,lb(7:8)];
    ub1=[ub1,ub(1:3)];
    ub2=[ub2,ub(4:6)];
    ub3=[ub3,ub(7:8)];
end
lb=[lb1,lb2,lb3];
ub=[ub1,ub2,ub3];
Z0=0;

%% plot
obstacles=0;%[[5,4],[4,6]];

xcont = linspace(x0(1),xr(1)+5);
ycont = linspace(x0(2)-5,xr(2)+5);
[X,Y] = meshgrid(xcont,ycont);
fun = (X-xr(1)).^2 + (Y-xr(2)).^2;
contour(X,Y,fun,100)
hold on 
for i=1:length(obstacles)/2
    scatter(obstacles(2*i-1),obstacles(2*i),'k','LineWidth',1.5)
end
grid on,set(gca,'ytick',min(xcont):max(xcont)),set(gca,'xtick',min(ycont):max(ycont))
% xline(3.5,'--r');
% xline(4.5,'--r');
% xline(5.5,'--r');

uk=u0;

for k = 2:Nsim+1 
    u_tilde(k-1,:)=(uk-ur);
    x_tilde(k-1,:)=(x(k-1,:)'-xr);
    [A,B] = Linearized_discrete_DD_model(xr,ur,dt); 
    [Z,fval,exitflag] = optimizer_fmincon(xr,Z0,A, B, N,x_tilde(k-1,:)',u_tilde(k-1,:)',lb,ub,obstacles);
%     obstacles(2)=obstacles(2)+0.05;
%     obstacles(4)=obstacles(4)-0.05;
    for i=1:length(obstacles)/2
    scatter(obstacles(2*i-1),obstacles(2*i),'k','LineWidth',1.5)
    end
    Z0=Z;
%     zz(k)=Z(5);
    %u(k-1,:)=Z(N*3+1:N*3+2)'+u_tilde(k-1,:)';
    u(k-1,:)=Z(N*3*2+1:N*3*2+2)'+ur;
    uk=u(k-1,:)';
    x(k,:) =A*x(k-1,:)' + B*u(k-1,:)';
 
    if exitflag==-2
        disp("----unfeasible!----")
       break
    end
    plot(x(:,1),x(:,2),'ok')
    pause(0.1)
end
figure(2)
subplot(2,1,1)
plot(x_tilde)
legend('x','y','theta')
title("x_bar")
subplot(2,1,2)
plot(u_tilde)
legend('u1','u2')
title("u_bar")
disp("done")


    
