function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs,xr,r_safety_margin,dist_cond,lines_st,lanewidth,lane_offset_x,lane_offset_y,lane_border_min,road_radius,obstacles_lanes,plot_x_curv,plot_y_curv,obstacle_radius)
ceq=zeros(N,1);

%% Model constraint
for i=1:N
    [A,B] = Linearized_discrete_DD_model(xk,uk,dt);
    xk1=Z(N*3+3*i-2:N*3+3*i)';
    ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
    xk=xk1;
    uk=Z(N*8+2*i-1:N*8+2*i)';
end

%% COLLISION CONE:
iter = 1;
cin=zeros(length(obstacles)*N,1);
obstacles_save = [];
r_tot = r_obs + r_safety_margin;
for j=1:length(obstacles)
    x_iter = 3*N+1;
    y_iter = 3*N+2;
    th_iter = 3*N+3;
    obstacle_comp = obstacles{j}(:);
    obstacle_comp_p1 = obstacle_comp; % initialize obstacle_comp
    for i = 1:N
        %% EASY TO READ VARIABLE UPDATE
        u = Z(8*N+1+(2*(i-1)));
        x = Z(x_iter);
        y = Z(y_iter);
        th = Z(th_iter);
        vx = sin(th)*u;
        vy = cos(th)*u;
        obstacle_comp = obstacle_comp_p1; % Update comp
        x_obs = obstacle_comp(1);
        y_obs = obstacle_comp(2);
        th_obs = obstacle_comp(3);
        v_obs = obstacles_u{j}(1); % Constant
        w_obs = obstacles_u{j}(2); % Constant
        
        % One step:
        [A,B] = Linearized_discrete_DD_model(obstacle_comp,[v_obs; w_obs],dt);
        obstacle_comp_p1 = A*[x_obs; y_obs; th_obs] + B*[v_obs; w_obs]; % Take one step to find p2 
        x_obs_p1 = obstacle_comp_p1(1);
        y_obs_p1 = obstacle_comp_p1(2);
        th_obs_p1 = obstacle_comp_p1(3);
        P = obstacle_comp_p1-obstacle_comp;
        V = P/dt; % Behövs denna? Storleken på v_obs ändras nu när vi inkluderar w i vektorerna vx och vy
       
        %% Find the true vx_obs and vy_obs
        vx_obs = (v_obs)*cos(obstacle_comp_p1(3));
        vy_obs = (v_obs)*sin(obstacle_comp_p1(3));
        
        
        
        ego_dist = sqrt((x-xr(1))^2+(y-xr(2))^2);
        obs_dist = sqrt((x_obs-xr(1))^2+(y_obs-xr(2))^2);
        
        %% Conditions:
        deactivated_cone = cone_logic(obs_dist, ego_dist, u, v_obs, x, y, x_obs, y_obs, dist_cond);
        
        if ~deactivated_cone
            %% Formulas:
            rterm = sqrt((x-x_obs)^2+(y-y_obs)^2); %OK
            uterm = ((cos(th)*u-vx_obs)*(x-x_obs) + (sin(th)*u-vy_obs)*(y-y_obs))^2;
            lterm = sqrt((cos(th)*u-vx_obs)^2+(sin(th)*u-vy_obs)^2); % Dividerar med
            cone = r_tot^2*lterm^2 - rterm^2*lterm^2 + uterm; %-(rterm-r_tot^2 - uterm/lterm);
            
            %% Check if traj or ref point
            if length(xr) > 3 %% ONLY FOR TRAJ!   % Use boolean instead.
                ego_dist = norm([Z(x_iter) Z(y_iter)]-[xr(1,i); xr(2,i)]);
                obst_dist = norm([obstacles{j}(1) obstacles{j}(2)]-[xr(1,i); xr(2,i)]);
            else    %% ONLY FOR REF POINT!
                ego_dist = sqrt((x-xr(1))^2 + (y-xr(2))^2);
                obst_dist = sqrt((x_obs-xr(1))^2 + (y_obs-xr(2))^2);
            end
            
            cin(iter,1) = cone; %((r_tot)^2)*norm(vab)^2-((norm(r)^2*norm(vab)^2)-dot(r,vab)^2);
        end
        
        %% Update obstacle state:
%         [A,B] = Linearized_discrete_DD_model(obstacle_comp,[v_obs; w_obs],dt);
        obstacle_comp = A*obstacle_comp + B*[v_obs; w_obs];
        %% Update iters:
        x_iter = x_iter + 3;
        y_iter = y_iter + 3;
        th_iter = th_iter + 3;
        iter = iter + 1;
    end
% cin = [];
% ceq = [];
end

end