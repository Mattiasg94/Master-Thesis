clc
clear all, close all

N = 10;
Nsim=100;
dt=0.2;
ur=[0.2;0];
xr=[10;5;0];

follow_traj = true; 
Y_REF = [11, 10.99, 10.96, 10.91, 10.84, 10.76, 10.65, 10.53, 10.39, 10.24, 10.08, 9.91, 9.72, 9.53, 9.34, 9.14, 8.94, 8.74, 8.55, 8.35, 8.17, 7.99, 7.82, 7.67, 7.53, 7.4, 7.29, 7.19, 7.12, 7.06, 7.02, 7.0, 7.0, 7.03, 7.07, 7.13, 7.21, 7.3, 7.42, 7.55, 7.69, 7.85, 8.02, 8.2, 8.39, 8.58, 8.78, 8.98, 9.17, 9.37, 9.57, 9.76, 9.94, 10.11, 10.27, 10.42, 10.55, 10.67, 10.77, 10.85, 10.92, 10.97, 10.99];
X_REF = [0.0, 0.16, 0.32, 0.48, 0.65, 0.81, 0.97, 1.13, 1.29, 1.45, 1.61, 1.77, 1.94, 2.1, 2.26, 2.42, 2.58, 2.74, 2.9, 3.06, 3.23, 3.39, 3.55, 3.71, 3.87, 4.03, 4.19, 4.35, 4.52, 4.68, 4.84, 5.0, 5.16, 5.32, 5.48, 5.65, 5.81, 5.97, 6.13, 6.29, 6.45, 6.61, 6.77, 6.94, 7.1, 7.26, 7.42, 7.58, 7.74, 7.9, 8.06, 8.23, 8.39, 8.55, 8.71, 8.87, 9.03, 9.19, 9.35, 9.52, 9.68, 9.84, 10.0];
THETA_REF=[0.0, -0.07841, -0.156035, -0.2321, -0.3058475, -0.37654, -0.44347, -0.5059675, -0.56341, -0.6152225, -0.66089, -0.6999525, -0.7320225, -0.7567775, -0.77397, -0.78343, -0.7850625, -0.7788525, -0.7648575, -0.7432225, -0.71416, -0.6779625, -0.6349925, -0.585675, -0.5305075, -0.47004, -0.404875, -0.3356625, -0.2631, -0.187905, -0.110835, -0.0326575, 0.0458475, 0.1238925, 0.2007025, 0.275505, 0.347555, 0.4161325, 0.4805525, 0.54017, 0.5943925, 0.6426725, 0.684535, 0.719555, 0.7473875, 0.76775, 0.7804425, 0.7853375, 0.782385, 0.7716175, 0.7531375, 0.7271325, 0.6938625, 0.6536625, 0.6069275, 0.55413, 0.495795, 0.4325075, 0.3648975, 0.2936425, 0.2194525, 0.14307, 0.0652575];


x0=[0.2;5;0];
u0=[0.1;0];
lb_x=[0 0 -2*pi];
ub_x=[inf inf 2*pi];
lb_u=[0 -0.5];
ub_u=[2 0.5];
dv=0.5;
dw=1;
lb=[-inf -inf -inf lb_x -inf -inf lb_u];
ub=[ inf  inf  inf ub_x  inf  inf ub_u];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta]=setup(x0,u0,xr,ur,ub,lb,Nsim,N);

%% plot
obstacles={}; % {[4;5;pi/6],[5;4.5;pi/6]};
obstacles_u={}; % {[0.5;0],[-0.5;0]};
plot_obstacles = plot(1); textbox=plot(1); plot_obstacles_radius=plot(1);
r_obs=0.8;
xcont = linspace(x0(1),xr(1)+5);
ycont = linspace(x0(2)-5,xr(2)+5);
[X,Y] = meshgrid(xcont,ycont);
fun = (X-xr(1)).^2 +(Y-xr(2)).^2;
contour(X,Y,fun,100)

% scatter(xr(1),xr(2),50,'y','LineWidth',5);
viscircles([xr(1),xr(2)],0.1,'Color','y','Linewidth',5);
xlim([0,22])
ylim([0,11])
hold on

Vfval=zeros(Nsim,1);
Vfval(1)=inf;
feasible=1;
close_obstacles=obstacles;
close_obstacles_u=obstacles_u;

for k = 1:Nsim+1
    if feasible
        [A,B] = Linearized_discrete_DD_model(x(k,:)',u(k,:)',dt);
        x(k+1,:) =    x(k,:)' +B*u(k,:)';
        if follow_traj
            u_tilde(k,:)= (u(k,:)'-ur);
            x_tilde(k,:)= (x(k,:)'-[X_REF(k); Y_REF(k); THETA_REF(k)]);
        else
            u_tilde(k,:)= (u(k,:)'-ur);
            x_tilde(k,:)= (x(k,:)'-xr);
        end
        delete(textbox)
    else
        delete(textbox)
        textbox=annotation('textbox', [0.7, 0.1, 0.1, 0.1], 'String', "Wait");
        u(k,:)=u(k-1,:)/10;
        x(k+1,:) = x(k,:)' + B*u(k,:)';
    end
    if follow_traj
        [Z,fval,exitflag] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
            ,N,lb,ub,close_obstacles,close_obstacles_u,r_obs,[X_REF;Y_REF;THETA_REF]);
    else
        [Z,fval,exitflag] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
            ,N,lb,ub,close_obstacles,close_obstacles_u,r_obs,xr);
    end
    
    u(k+1,:)=Z(N*8+1:N*8+2)';
    Z0=Z;
    Zx_tilde=Z(1:3*N);
    Zx=Z(3*N+1:6*N);
    for j=1:N
        Zx_plot(j,:)=Zx(3*j-2:3*j);
        Zx_tilde_plot(j,:)=Zx_tilde(3*j-2:3*j);
    end
    
    plot(x(:,1),x(:,2),'.k')
    xlim([0,11])
    ylim([0,11])
    track=plot(Zx_plot(:,1),Zx_plot(:,2),'*g');
    delete(plot_obstacles)
    delete(plot_obstacles_radius)
    j=1;
    close_obstacles={};
    close_obstacles_u={};
    for i=1:length(obstacles)
        %plot_obstacles(i) = scatter(obstacles{i}(1),obstacles{i}(2),'g','LineWidth',1);
        plot_obstacles(i)=viscircles([obstacles{i}(1),obstacles{i}(2)],0.1,'Color','r','Linewidth',2);
        plot_obstacles_radius(i)=viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs,'LineStyle','--','Color','r','Linewidth',0.8);
        [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{i},obstacles_u{i},dt);
        obstacles{i}=A_obstacles*obstacles{i}+B_obstacles*obstacles_u{i};
        if abs(x(k+1,1)-obstacles{i}(1))+abs(x(k+1,2)-obstacles{i}(2))<10
            %             disp('--- Obstacle visible! ---')
            close_obstacles{j}=obstacles{i};
            close_obstacles_u{j}=obstacles_u{i};
            j=j+1;
            %         elseif isempty(close_obstacles) % Needed to use IPOPT
            %             close_obstacles{j}=obstacles{1};
            %             close_obstacles_u{j}=obstacles_u{1};
        end
    end
    
    pause(0.1)
    delete(track)
    
    Vfval(k+1)=fval;
    test(k)=Vfval(k)-fval;
    last_dist_xr=abs(Z(6*N-2)-xr(1));
    last_dist_yr=abs(Z(6*N-1)-xr(2));
    
    if abs(x(k+1,1)-xr(1))<0.2 && abs(x(k+1,2)-xr(2))<0.2 && abs(x(k+1,3)-xr(3))<0.2
        break
    end
    if exitflag==-2
        if  Vfval(k)-fval<=225
            disp("----converge slowly----")
        end
        if last_dist_xr>0.1 && last_dist_yr>0.1
            disp("----Unfeasible!----")
            feasible=0;
        end
        continue
    else
        feasible=1;
    end
    
end

% figure(2)
% plot(Vfval)
% figure(3)
% plot(test)
% subplot(2,1,1)
% plot(x_tilde)
% legend('x','y','theta')
% title("x_bar")
% subplot(2,1,2)
% plot(u_tilde)
% legend('u1','u2')
% title("u_bar")




