clc
clear all, close all

N=5;
Nsim=50;
dt=0.1;
ur=[0.1;0.1];
xr=[10;10;0];
x0=[4;6;0];
u0=[0;0];
lb_x=[0 0 -2*pi];
ub_x=[20 20 2*pi];
lb_u=[-1 -2*pi];
ub_u=[10 2*pi];
lb=[-inf -inf -inf lb_x -inf -inf lb_u];
ub=[ inf  inf  inf ub_x  inf  inf ub_u];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu_delta]=setup(x0,xr,ur,ub,lb,Nsim,N);
%% plot
obstacles=0;[[6,3],[0,0]];
xcont = linspace(0,xr(1)+5);
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
    [A,B] = Linearized_discrete_DD_model(x(k-1,:)',uk,dt);
    
    [Z,fval,exitflag] = optimizer_fmincon(dt,xr,Z0,A, B,MQ,MR,Mxr,Mur,N,...
                     x_tilde(k-1,:)',u_tilde(k-1,:)',x(k-1,:)',lb,ub,obstacles,x0,u0);
    
                 
                 
    for i=1:length(obstacles)/2
        scatter(obstacles(2*i-1),obstacles(2*i),'k','LineWidth',1.5)
    end
    Z0=Z;
    u(k-1,:)=Z(N*3*2+1:N*3*2+2)'+ur;
    uk=u(k-1,:)';
    x(k,:) =A*x(k-1,:)' + B*u(k-1,:)';
    if exitflag==-2
        disp("----Infeasible!----")
        break
    end
    plot(x(:,1),x(:,2),'ok')
    pause(0.01)
    for i=1:N-1
        test(k)=fval;
    end
end

figure(2)
plot(test)
title('Fval')
figure(3)
subplot(2,1,1)
plot(x_tilde)
legend('x','y','theta')
title("x_{bar}")
subplot(2,1,2)
plot(u_tilde)
legend('u1','u2')
title("u_{bar}")



