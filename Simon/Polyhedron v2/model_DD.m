clc
clear all, close all

N = 10;
Nsim=60;
dt=0.2;
ur=[0.2;0];
xr=[10;5;0];
x0=[0.2;5;0];
u0=[0.1;0];
lb_x=[0 0 -2*pi];
ub_x=[10 10 2*pi];
lb_u=[0.1 -0.5];
ub_u=[2 0.5];
dv=0.5;
dw=0.5;
lb=[lb_x lb_u];
ub=[ub_x ub_u];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta]=setup(x0,u0,xr,ur,ub,lb,Nsim,N);
Zr = [];
for i =1:N
    Zr = [Zr; xr];
end
for i =1:N    
    Zr = [Zr; ur];
end


%% plot
obstacles={[4;5;0]}; %,[5;4.5;0]};
obstacles_u={[0.1;0]}; %,[0.25;0]};
plot_obstacles = plot(1); textbox=plot(1); plot_obstacles_radius=plot(1);
r_obs=0.5;
xcont = linspace(x0(1),xr(1)+5);
ycont = linspace(x0(2)-5,xr(2)+5);
[X,Y] = meshgrid(xcont,ycont);
fun = (X-xr(1)).^2 +(Y-xr(2)).^2;
contour(X,Y,fun,100)

scatter(xr(1),xr(2),50,'y','LineWidth',5);
viscircles([xr(1),xr(2)],0.1,'Color','y','Linewidth',5);
xlim([0,11])
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
        x(k+1,:) =A*x(k,:)' + B*u(k,:)';
        delete(textbox)
    else
        delete(textbox)
        textbox=annotation('textbox', [0.7, 0.1, 0.1, 0.1], 'String', "Wait");
        u(k,:)=u(k-1,:)/2;
        x(k+1,:) =A*x(k,:)' + B*u(k,:)';
    end
    [Z,fval,exitflag] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
        ,N,lb,ub,close_obstacles,close_obstacles_u,r_obs,Zr);
    u(k+1,:)=Z(N*3+1:N*3+2)';
    Z0=Z;
    Zx=Z(1:3*N);
    for j=1:N
        Zx_plot(j,:)=Zx(3*j-2:3*j);
    end

    plot(x(:,1),x(:,2),'ok')
    xlim([0,11])
    ylim([0,11])
    track=plot(Zx_plot(:,1),Zx_plot(:,2),'*g');
    delete(plot_obstacles)
    delete(plot_obstacles_radius)
    j=1;
    close_obstacles={};
    close_obstacles_u={};
%     for i=1:length(obstacles)
%         %plot_obstacles(i) = scatter(obstacles{i}(1),obstacles{i}(2),'g','LineWidth',1);
%         plot_obstacles(i)=viscircles([obstacles{i}(1),obstacles{i}(2)],0.1,'Color','r','Linewidth',2);
%         plot_obstacles_radius(i)=viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs,'LineStyle','--','Color','r','Linewidth',0.8);
%         [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{i},obstacles_u{i},dt);
%         obstacles{i}=A_obstacles*obstacles{i}+B_obstacles*obstacles_u{i};
%         if abs(x(k+1,1)-obstacles{i}(1))+abs(x(k+1,2)-obstacles{i}(2))<5
%             close_obstacles{j}=obstacles{i};
%             close_obstacles_u{j}=obstacles_u{i};
%             j=j+1;
%         elseif isempty(close_obstacles) % Needed to use IPOPT
%             close_obstacles{j}=obstacles{1};
%             close_obstacles_u{j}=obstacles_u{1};
%         end
%     end
    
    pause(0.000001)
    delete(track)
    
    Vfval(k+1)=fval;
    test(k)=Vfval(k)-fval;
    last_dist_xr=abs(Z(3*N-2)-xr(1));
    last_dist_yr=abs(Z(3*N-1)-xr(2));
    
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




