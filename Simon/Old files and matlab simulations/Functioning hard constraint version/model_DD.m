clc
clear all, close all

N=10;
Nsim=50;
dt=0.5;
ur=[0.1;0]; 
xr=[5;10;0];
x0=[5;0;0];
u0=[1;0];
x0_ub = [10    10  2*pi];
x0_lb = [-10  -10 -2*pi];
u0_ub = [ 1 1];
u0_lb = [ -1 -1];
lb=[-inf -inf -inf x0_lb u0_lb];
ub=[inf inf inf x0_ub u0_ub];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR]=setup(x0,ub,lb,Nsim,N);
%% plot
obstacles=[[5,5],[4,6]];
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
xline(3.5,'--r');
xline(4.5,'--r');
xline(5.5,'--r');

uk=u0;
for k = 2:Nsim+1 
    u_tilde(k-1,:)=(uk-ur);
    x_tilde(k-1,:)=(x(k-1,:)'-xr);
    [A,B] = Linearized_discrete_DD_model(xr,ur,dt); 
    [Z,fval,exitflag] = optimizer_fmincon(xr,Z0,A, B,MQ,MR, N,x_tilde(k-1,:)',u_tilde(k-1,:)',lb,ub,obstacles);
    if k>=29
        obstacles(1)=obstacles(1)+0.05;
    else
        obstacles(2)=obstacles(2)+0.05;
    end
    obstacles(4)=obstacles(4)-0.05;
    for i=1:length(obstacles)/2
        scatter(obstacles(2*i-1),obstacles(2*i),'k','LineWidth',1.5)
    end
    Z0=Z;
    u(k-1,:)=Z(N*3*2+1:N*3*2+2)'+ur;
    uk=u(k-1,:)';
    x(k,:) =A*x(k-1,:)' + B*u(k-1,:)';
    
    obstacleDist = sqrt((x(k,1)-obstacles(1))^2 + (x(k,2)-obstacles(2))^2)
    
    if exitflag==-2
       disp("----Unfeasible!----")
       break
    end
    plot(x(:,1),x(:,2),'ok')
    delete(findall(gcf,'type','annotation'))
    annotation('textbox', [0.5, 0.2, 0.1, 0.1], 'String', "Iteration:" + k)
    pause(0.001)
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



    
