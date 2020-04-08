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
    Mxr = [];
    for i = 1:N
        % Mxr = [Mxr;[X_REF(k+i);Y_REF(k+i);THETA_REF(k+i)]];
        Mxr = [Mxr;[xr(1);xr(2);xr(3)]];
    end

    % Feasible
    if Error_is_small
        [A,B] = Linearized_discrete_DD_model(x(k,:)',u(k,:)',dt);
        x(k+1,:) =    x(k,:)' +B*u(k,:)';
        u_tilde(k,:)= (u(k,:)'-ur);
        x_tilde(k,:)= (x(k,:)'-xr);
        
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
            warmstart = false;
        end
    end
    warmstart = true; % WARMSTART ALWAYS ON IF TRUE!
    
    [Z,fval,exitflag,timerVal] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
        ,N,lb,ub,obstacles,obstacles_u,r_obs,xr,lines_st,MR_jerk,r_safety_margin,lane_offset_x,lane_offset_y,lane_border_max,lane_border_min,lanewidth,dist_cont,road_radius,obstacles_lanes,plot_x_curv,plot_y_curv,obstacle_radius,warmstart,center);
    
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
    
    %% Impact check
    impact.x = [];
    impact.y = [];
    for i=1:length(obstacles)
        plot_obstacles(i) = viscircles([obstacles{i}(1),obstacles{i}(2)],0.1,'Color','r','Linewidth',2);
        plot_obstacles_radius(i) = viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs,'LineStyle','--','Color','r','Linewidth',0.8);
        plot_obstacles_safety_radius(i) = viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs+r_safety_margin,'LineStyle','--','Color','r','Linewidth',0.8);
        
        %% FIX THIS!
%         [A,B] = Linearized_discrete_DD_model(obstacles{i},obstacles_u{i},dt);
%         obstacles{i} = A*obstacles{i} + B*obstacles_u{i};
        [obstacles{i}(1), obstacles{i}(2)] = obs_move_line(dt, lines_st.lanes(i), obstacles_u{i}(1), obstacles{i}(1), obstacles{i}(2), center,road_radius,lanewidth);
        obstacles{i}(2) = get_y_from_lane(obstacles_lanes{i}, obstacles{i}(1),plot_x_curv,plot_y_curv,lanewidth);
%         v_init = u(1);
%         x_init = Z(3*N+1);  %% Dessa kan beh�va g�ras om
%         y_init = Z(3*N+2);
%         theta_init = Z(3*N+3);
%         Omega_ego=get_tang_v_ego(v_init,x_init,y_init,theta_init);
%         t_impact=get_intersection_time(x_init,y_init,Omega_ego,x_obs(i),y_obs(i),w_obs(i));
%         [x_imp,y_imp,~,~]=obs_move_line(dt,lane, v, x_obs(i), y_obs(i), road_radius, lanewidth, lane_offset_x, lane_offset_y, lane_border_min,center);
%         impact.x = [x_impact x_imp];
%         impact.y = [y_impact y_imp];
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
        warmstart = false;
        %         iterSaved_u = iterSaved_u + 1;
    end
end

disp('Mean optimizer time: ')
disp(mean(timerSave))