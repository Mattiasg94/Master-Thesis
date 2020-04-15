%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main file                                                               %
% Runs MPC and calls the optimization builder.                            %
% The builder constructs objective functions and constraint functions and %
% calls the chosen optimizer.                                             %
% Things to add/do here:                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% !!!! Run from initFile.m !!!!
%% Simulate MPC
for k = 1:Nsim
    % MXR is needed for traj.
    Mxr = [];
    [xr(1), xr(2)] = move_ref_point( x(k,1), x(k,2), xref_final, yref_final, reference_lane_number, center, ub_u(1)*N*dt+round(0.33*N), road_radius, lanewidth);
%     xr(1) = xref_final;
%     xr(2) = yref_final;
    plot(xr(1),xr(2),'.b');
    
    for i = 1:N
        % Mxr = [Mxr;[X_REF(k+i);Y_REF(k+i);THETA_REF(k+i)]];
        Mxr = [Mxr;[xr(1);xr(2);xr(3)]];
    end
    

    % NOTE!!!!!!!!!!!
    warmstart = true;
    if x(k,1) > 20
        warmstart = false;
    end
    
    [Z,fval,exitflag,timerVal] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,...
        dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta,N,lb,ub,obstacles,...
        obstacles_u,r_obs,xr,MR_jerk,r_safety_margin,...
        lane_border_min,lanewidth,dist_cont,road_radius,...
        obstacles_lanes,warmstart,center,barrier_weight,Qt,options);
    Z0 = Z;
    Zx = Z(3*N+1:6*N);
    
    for j = 1:N
        Zx_plot(j,:) = Zx(3*j-2:3*j);
    end
    hold on
    track = plot(Zx_plot(:,1),Zx_plot(:,2),'*g');
    pause(0.1)
    
    u(k+1,:) = Z(N*8+1:N*8+2);
    
    if u(k+1,1)<0.1 && Z(N*8+1+2:N*8+1+2)>0.2 && Z(N*8+1+2+2:N*8+1+2+2)>0.7 && Z(N*8+1+2+2+2:N*8+1+2+2+2)>1.2
        u(k+1,1)=2;
        disp("stuck")
    end
    
    [A,B] = Linearized_discrete_DD_model(x(k,:)',u(k,:)',dt);
    x(k+1,:) =    x(k,:)' +B*u(k,:)';
    u_tilde(k,:)= (u(k,:)'-ur);
    x_tilde(k,:)= (x(k,:)'-xr);
    timerSave(k) = timerVal;
    hold on
    plot(x(:,1),x(:,2),'.k')
%     pause(1)
    delete(plot_obstacles)
    delete(plot_obstacles_radius)
    delete(plot_obstacles_safety_radius)
    delete(track)
    
    %% Check infeasibility
    %     [p,S] = polyfit(Zx_plot(:,1),Zx_plot(:,2),grade);
    %     t2 = min(Zx_plot(:,1)):(max(Zx_plot(:,1))-min(Zx_plot(:,1)))/(length(Zx_plot(:,1))-1):max(Zx_plot(:,1));
    %     [y2,delta] = polyval(p,t2,S);
    %     if length(Zx_plot(:,2)) == length(y2(:))    % TODO
    %         Err = immse(Zx_plot(:,2),y2(:));
    %     else
    %         Err = 0; % Reached goal if lengths are missmatched.
    %     end
    
    %% Impact check
    for i = 1:length(obstacles)
        if ~isempty(impactPlot)
            delete(impactPlot(i))
        end
    end
    for i=1:length(obstacles)
        plot_obstacles(i) = viscircles([obstacles{i}(1),obstacles{i}(2)],0.1,'Color','r','Linewidth',2);
        plot_obstacles_radius(i) = viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs,'LineStyle','--','Color','r','Linewidth',0.8);
        plot_obstacles_safety_radius(i) = viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs+r_safety_margin,'LineStyle','--','Color','r','Linewidth',0.8);
        
        if scenario4 && obstacles{1}(1)<0      % Criterion to use sudden_obs
            [obstacles{i}(1),obstacles{i}(2),x_sudden_detect,y_sudden_detect] = place_sudden_obs(x(k,1),x(k,2),obstacles{i}(1),obstacles{i}(2),obstacles_lanes{i},plot_x_curv,plot_y_curv,lanewidth);
        else                                   % NOT sudden_obs
            [obstacles{i}(1), obstacles{i}(2)] = obs_move_line(dt, obstacles_lanes{i}, obstacles_u{i}(1), obstacles{i}(1), obstacles{i}(2), center,road_radius,lanewidth);
            obstacles{i}(2) = get_y_from_lane(obstacles_lanes{i}, obstacles{i}(1),plot_x_curv,plot_y_curv,lanewidth);
        end
        %% MATTIAS KOLLA H�R:
        v_init = Z(8*N+1+2); % u(k+1,1);
        x_init = Z(3*N+1+3);
        y_init = Z(3*N+2+3);
        theta_init = Z(3*N+3+3);
        v_tang_ego=get_tang_v_ego(v_init,x_init,y_init,theta_init,center); %(v,x,y,th,center)
        r_circ_ego = sqrt( (x_init - center(1))^2 + (y_init-center(2))^2 );
        r_circ_obs = road_radius_frm_lane(obstacles_lanes{i},road_radius,lanewidth); % sqrt( (obstacles{i}(1) - center(1))^2 + (obstacles{i}(2)-center(2))^2 );
        [t_impact, ~]=get_intersection_time(x_init,y_init,v_tang_ego,obstacles{i}(1),obstacles{i}(2),obstacles_u{i}(1),r_circ_ego,r_circ_obs,center);
        [x_imp,y_imp,~,~]=obs_move_line(t_impact,obstacles_lanes{i}, obstacles_u{i}(1), obstacles{i}(1), obstacles{i}(2),center, road_radius, lanewidth);

        hold on
        impactPlot(i) = plot(x_imp,y_imp,'yo','MarkerSize',5, 'MarkerFaceColor','k'); % Plot collision obs
        
    end
    
    %% Save the previous input vector
%     if Err <= 0.091 % Iteratively found
%         warmstart = true;
%         Error_is_small = true;
%         u_saved = reshape(Z(N*8+1:end),[2 2*N/2]);
%     else
%         Error_is_small = false;
%         warmstart = false;
%     end
%     
%     pause(0.00001)
%     delete(track)
    
    %% Control if close to ref
    if abs(x(k+1,1)-xr(1))<0.2 && abs(x(k+1,2)-xr(2))<0.2 && abs(x(k+1,3)-xr(3))<0.2
        break
    end
    
end

disp('Average optimizer time: ')
disp(mean(timerSave))