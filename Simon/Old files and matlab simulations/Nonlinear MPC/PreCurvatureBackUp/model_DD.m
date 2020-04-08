clc
clear all, close all

initFile;

for k = 1:Nsim+1
    % MXR is needed for traj.
    if follow_traj
        Mxr = [];
        for i = 1:N
            if k+i >= length(X_REF)
                Mxr =[Mxr; [X_REF(end); Y_REF(end); THETA_REF(end)]];
            else
                Mxr = [Mxr;[X_REF(k+i);Y_REF(k+i);THETA_REF(k+i)]];
            end
        end
    end
    
    if feasible
        [A,B] = Linearized_discrete_DD_model(x(k,:)',u(k,:)',dt);
        x(k+1,:) =    x(k,:)' +B*u(k,:)';
        if follow_traj
            u_tilde(k,:)= (u(k,:)'-ur);
            if k >= length(X_REF)
                x_tilde(k,:)= (x(k,:)'-[X_REF(end); Y_REF(end); THETA_REF(end)]);
            else
                x_tilde(k,:)= (x(k,:)'-[X_REF(k); Y_REF(k); THETA_REF(k)]);
            end
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
        [Z,fval,exitflag,timerVal] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
            ,N,lb,ub,close_obstacles,close_obstacles_u,r_obs,[X_REF;Y_REF;THETA_REF],lane_st,ylimit,MR_jerk,r_safety_margin);
    else
        [Z,fval,exitflag,timerVal] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
            ,N,lb,ub,close_obstacles,close_obstacles_u,r_obs,xr,lane_st,ylimit,MR_jerk,r_safety_margin);
    end
    timerSave(k) = timerVal;
    u(k+1,:)=Z(N*8+1:N*8+2)';
    Z0=Z;
%     Zx_tilde=Z(1:3*N);
    Zx=Z(3*N+1:6*N);
    for j=1:N
        Zx_plot(j,:)=Zx(3*j-2:3*j);
%         Zx_tilde_plot(j,:)=Zx_tilde(3*j-2:3*j);
    end
    
    plot(x(:,1),x(:,2),'.k')
    hold on
    track=plot(Zx_plot(:,1),Zx_plot(:,2),'*g');
    delete(plot_obstacles)
    delete(plot_obstacles_radius)
    delete(plot_obstacles_safety_radius)
   
    j=1;
    close_obstacles={};
    close_obstacles_u={};
    for i=1:length(obstacles)
        plot_obstacles(i)=viscircles([obstacles{i}(1),obstacles{i}(2)],0.1,'Color','r','Linewidth',2);
        plot_obstacles_radius(i)=viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs,'LineStyle','--','Color','r','Linewidth',0.8);
        plot_obstacles_safety_radius(i)=viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs+r_safety_margin,'LineStyle','--','Color','r','Linewidth',0.8);
       
        [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{i},obstacles_u{i},dt);
        obstacles{i}=A_obstacles*obstacles{i}+B_obstacles*obstacles_u{i};
        if abs(x(k+1,1)-obstacles{i}(1))+abs(x(k+1,2)-obstacles{i}(2))<10
            close_obstacles{j}=obstacles{i};
            close_obstacles_u{j}=obstacles_u{i};
            j=j+1;
        end
    end
    
    pause(0.001)
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
        %% FUNGERAR EJ!!
        if last_dist_xr>0.01 && last_dist_yr>0.01
            disp("----Infeasible!----")
            feasible=0; % Skall egentligen vara = 0
        end
        continue
    else
        feasible=1;
    end
    
end

disp('Mean optimizer time: ')
disp(mean(timerSave))


