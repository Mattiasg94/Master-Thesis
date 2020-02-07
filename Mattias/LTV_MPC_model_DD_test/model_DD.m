clc
clear all, close all

N=10;
Nsim=35;
dt=0.5;
ur=[0;0]; 
xr=[10;5;0];
x0=[0;2;-pi/8];
u0=[0;0];
lb_x=[0 0 -2*pi];
ub_x=[10 10 2*pi];
lb_u=[0 -0.5];
ub_u=[2 0.5];
dv=0.5;
dw=0.1;
lb=[-inf -inf -inf lb_x -inf -inf lb_u];
ub=[ inf  inf  inf ub_x  inf  inf ub_u];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta]=setup(x0,u0,xr,ur,ub,lb,Nsim,N);
%% plot
obstacles={[0;2.5;0],[1;2.5;0]};
obstacles_u={[1;0],[1;0]};
plot_obstacles = plot(1); textbox=plot(1);

xcont = linspace(x0(1),xr(1)+5);
ycont = linspace(x0(2)-5,xr(2)+5);
[X,Y] = meshgrid(xcont,ycont);
fun = (X-xr(1)).^2 +(Y-xr(2)).^2;
contour(X,Y,fun,100)
hold on 
%grid on,set(gca,'ytick',min(xcont):max(xcont)),set(gca,'xtick',min(ycont):max(ycont))

Vfval=zeros(Nsim,1);
Vfval(1)=inf;
feasible=1;
for k = 1:Nsim+1 
    if feasible
        [A,B] = Linearized_discrete_DD_model(x(k,:)',u(k,:)',dt);    
        x(k+1,:) =A*x(k,:)' + B*u(k,:)';
        u_tilde(k,:)=(u(k,:)'-ur);
        x_tilde(k,:)=(x(k,:)'-xr);
        delete(textbox)
    else
       delete(textbox)
       textbox=annotation('textbox', [0.7, 0.1, 0.1, 0.1], 'String', "Wait");
        u(k,:)=u(k-1,:)/2;
        x(k+1,:) =A*x(k,:)' + B*u(k,:)';
    end
    [Z,fval,exitflag] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
        ,N,lb,ub,obstacles);
    u(k+1,:)=Z(N*8+1:N*8+2)';
    Z0=Z;
    Zx_tilde=Z(1:3*N);
    Zx=Z(3*N+1:6*N);
    for j=1:N
        Zx_plot(j,:)=Zx(3*j-2:3*j);
        Zx_tilde_plot(j,:)=Zx_tilde(3*j-2:3*j);        
    end    
    
    plot(x(:,1),x(:,2),'ok')
    track=plot(Zx_plot(:,1),Zx_plot(:,2),'*r');
    delete(plot_obstacles)
    for i=1:length(obstacles)
        plot_obstacles(i) = scatter(obstacles{i}(1),obstacles{i}(2),'k','LineWidth',1.5);
        [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{i},obstacles_u{i},dt);
        obstacles{i}=A_obstacles*obstacles{i}+B_obstacles*obstacles_u{i};
    end
    pause(0.1)    
    delete(track)
    
    Vfval(k+1)=fval;
    test(k)=Vfval(k)-fval;
    last_dist_xr=abs(Z(6*N-2)-xr(1));
    last_dist_yr=abs(Z(6*N-1)-xr(2));
    if abs(x(k+1,1)-xr(1))<0.1 && abs(x(k+1,2)-xr(2))<0.1
        break
    end
    if exitflag==-2 || Vfval(k-1)-fval<=225
       if last_dist_xr>0.1 && last_dist_yr>0.1
           disp("----Unfeasible!----")
           feasible=0;
       end
       continue
    else
        feasible=1;
    end
    
end

figure(2)
plot(Vfval)
figure(3)
plot(test)
% subplot(2,1,1)
% plot(x_tilde)
% legend('x','y','theta')
% title("x_bar")
% subplot(2,1,2)
% plot(u_tilde)
% legend('u1','u2')
% title("u_bar")



    