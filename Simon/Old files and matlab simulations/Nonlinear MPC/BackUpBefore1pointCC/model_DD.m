clc;
clear all;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main file                                                               %
% Runs MPC and calls the optimization builder.                            %
% The builder constructs objective functions and constraint functions and %
% calls the chosen optimizer.                                             %
% Things to add/do here:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize MPC
initFile;

%% Simulate MPC
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
    
    % Feasible
    if Error_is_small
        [A,B] = Linearized_discrete_DD_model(x(k,:)',u(k,:)',dt);
        x(k+1,:) =    x(k,:)' +B*u(k,:)';
        if follow_traj
            u_tilde(k,:) = (u(k,:)'-ur);
            if k >= length(X_REF)
                x_tilde(k,:) = (x(k,:)'-[X_REF(end); Y_REF(end); THETA_REF(end)]);
            else
                x_tilde(k,:) = (x(k,:)'-[X_REF(k); Y_REF(k); THETA_REF(k)]);
            end
        else
            u_tilde(k,:)= (u(k,:)'-ur);
            x_tilde(k,:)= (x(k,:)'-xr);
        end
        delete(textbox)
        % NOT feasible
    else
        delete(textbox)
        textbox=annotation('textbox', [0.7, 0.1, 0.1, 0.1], 'String', "Use prev. input");
        if iterSaved_u <= length(u_saved(1,:))
            x(k+1,:) = x(k,:)' + B*u_saved(:,iterSaved_u);
            u(k,:) = u_saved(:,iterSaved_u);
            iterSaved_u = iterSaved_u + 1;
            warmstart = false;
        else
            x(k+1,:) = x(k,:)' + [0; 0; 0]; % Sust stop
            u(k,:) = [0 0];
            iterSaved_u = 1;
        end
    end
    warmstart = false; % WARMSTART ALWAYS ON IF TRUE!
    
    if follow_traj
        [Z,fval,exitflag,timerVal] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
            ,N,lb,ub,obstacles,obstacles_u,r_obs,[X_REF;Y_REF;THETA_REF],lines_st,MR_jerk,r_safety_margin,lane_offset_x,lane_offset_y,lane_border_max,lane_border_min,lanewidth,dist_cont,road_radius,obstacles_lanes,plot_x_curv,plot_y_curv,obstacle_radius,warmstart);
    else
        [Z,fval,exitflag,timerVal] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
            ,N,lb,ub,obstacles,obstacles_u,r_obs,xr,lines_st,MR_jerk,r_safety_margin,lane_offset_x,lane_offset_y,lane_border_max,lane_border_min,lanewidth,dist_cont,road_radius,obstacles_lanes,plot_x_curv,plot_y_curv,obstacle_radius,warmstart);
    end
    
    timerSave(k) = timerVal;
    u(k+1,:) = Z(N*8+1:N*8+2)';
    Z0 = Z;
    Zx = Z(3*N+1:6*N);
    
    for j = 1:N
        Zx_plot(j,:) = Zx(3*j-2:3*j);
    end
    
    plot(x(:,1),x(:,2),'.k')
    hold on
    track = plot(Zx_plot(:,1),Zx_plot(:,2),'*g');
    delete(plot_obstacles)
    delete(plot_obstacles_radius)
    delete(plot_obstacles_safety_radius)
    
    %% Check infeasibility
    
    [p,S] = polyfit(Zx_plot(:,1),Zx_plot(:,2),grade);
    t2 = min(Zx_plot(:,1)):(max(Zx_plot(:,1))-min(Zx_plot(:,1)))/(length(Zx_plot(:,1))-1):max(Zx_plot(:,1));
    [y2,delta] = polyval(p,t2,S);
    if length(Zx_plot(:,2)) == length(y2(:))
        Err = immse(Zx_plot(:,2),y2(:));
    else
        Err = 0; % Reached goal if lengths are missmatched.
    end
    
    
    for i=1:length(obstacles)
        plot_obstacles(i) = viscircles([obstacles{i}(1),obstacles{i}(2)],0.1,'Color','r','Linewidth',2);
        plot_obstacles_radius(i) = viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs,'LineStyle','--','Color','r','Linewidth',0.8);
        plot_obstacles_safety_radius(i) = viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs+r_safety_margin,'LineStyle','--','Color','r','Linewidth',0.8);
        
        %% FIX THIS!
        [A,B] = Linearized_discrete_DD_model(obstacles{i},obstacles_u{i},dt);
        obstacles{i} = A*obstacles{i} + B*obstacles_u{i};
%         [obstacles{i}(1), obstacles{i}(2)] = obs_move_line(dt, lines_st.lanes(i), obstacles_u{i}(1), obstacles{i}(1), obstacles{i}(2),road_radius,lanewidth,lane_offset_x,lane_offset_y,lane_border_min);
%         obstacles{i}(2) = get_y_from_lane(obstacles_lanes{i}, obstacles{i}(1),plot_x_curv,plot_y_curv,lanewidth);
    end
    
    pause(0.00001)
    delete(track)
    
    if abs(x(k+1,1)-xr(1))<0.2 && abs(x(k+1,2)-xr(2))<0.2 && abs(x(k+1,3)-xr(3))<0.2
        break
    end
    
    %% Save the previous input vector
    if Err <= 0.091 % Iteratively found
        warmstart = true;
        Error_is_small = true;
        u_saved = reshape(Z(N*8+1:end),[2 2*N/2]);
%         iterSaved_u = 0;
    else
        Error_is_small = false;
%         iterSaved_u = iterSaved_u + 1;
    end
end

disp('Mean optimizer time: ')
disp(mean(timerSave))