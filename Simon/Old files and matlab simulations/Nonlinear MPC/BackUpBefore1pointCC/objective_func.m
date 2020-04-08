function obj_fun = objective_func(dt,Z,MQ,MR,MR_jerk,N,obstacles,obstacles_u,lines_st,x_offset,y_offset,lane_border_max,lane_border_min,xr,lanewidth,dist_cond,obstacle_radius)
Z = Z(:); % Guarantee that Z is a colon vector

%% Initialize costs:
lane_cost = 0;
barrier_cost = 0;
lane_keep_cost = 0;
lineslope = 600;
deactivated_cone = [];
for i = 0:N-1
    % ---- Easy to read vars ----
    u = Z(8*N+1+(2*(i-1)));
    x = Z(3*N+1+3*i);
    y = Z(3*N+2+3*i);
    th = Z(3*N+3+3*i);
    radius_ego = sqrt((x-x_offset)^2 + (y+(lane_border_min - y_offset))^2);
    vx = sin(th)*u;
    vy = cos(th)*u;
    % ---------------------------
    %% Cost lane_cost
    lane1 = (lane_border_min+(lanewidth/2));
    lane2 = lane1+lanewidth;
    dist_ymin = 12*(lane1-radius_ego);
    dist_ymax = 12*(radius_ego-lane2);
    lane_cost = lane_cost + dist_ymin^3+10*dist_ymin;
    lane_cost = lane_cost + dist_ymax^3+10*dist_ymax;
    
    %% Barrier against the wall
%     if (lane_border_max-radius_ego) < lanewidth/2 %  >= (radius_ego-lane_border_min)
%         % Top
%         barrier_cost = barrier_cost + lineslope*sqrt((lane_border_max - radius_ego)^2)+lineslope/4;
%     elseif (radius_ego-lane_border_min) < lanewidth/2
%         % Bottom
%         barrier_cost = barrier_cost + lineslope*sqrt((lane_border_min - radius_ego)^2)+lineslope/4;
%     end
    for j = 1:length(obstacles)
        % ---- Easy to read vars, updated depending on obstacle ----
        x_obs = obstacles{j}(1);
        y_obs = obstacles{j}(2);
        v_obs = obstacles_u{j}(1);
        w_obs = obstacles_u{j}(2);
        [A,B] = Linearized_discrete_DD_model(obstacles{j}(:),[v_obs; w_obs],dt);
        obstacles_p1{j}(:) = A*obstacles{j}(:) + B*[v_obs; w_obs];
        P = obstacles_p1{j}(:)-obstacles{j}(:);
        V = P/dt; % Behövs denna? Storleken på v_obs ändras nu när vi inkluderar w i vektorerna vx och vy
        
        vx_obs = v_obs*cos(obstacles_p1{j}(3)); % norm(V)
        vy_obs = v_obs*sin(obstacles_p1{j}(3));
        ego_dist = sqrt((x-xr(1))^2+(y-xr(2))^2);
        obs_dist = sqrt((x_obs-xr(1))^2+(y_obs-xr(2))^2);
        
        %% Conditions:
        deactivated_cone(j) = cone_logic(obs_dist, ego_dist, u, v_obs, x, y, x_obs, y_obs, dist_cond);
        
        %% Update:
        obstacles{j}(:) = A*obstacles{j}(:) + B*[v_obs; w_obs];
    end
    if all(ismember(deactivated_cone,1)) %|| deactivated_cone == []  % If both cone for both obstacles is OFF, track reference lane!
        % This needs to be very large in order to converge back to the
        % reference lane.
        lane_keep_cost = lane_keep_cost + 25000*(radius_ego - lines_st.reference_lane)^2;
    else
        lane_keep_cost = lane_keep_cost + 35000*(radius_ego - 101)^2;
    end
    
end
obj_fun = Z(1:N*3)'*MQ*Z(1:N*3)+Z(N*6+1:N*8)'*MR*Z(N*6+1:N*8) + sum(MR_jerk(end-1:end-1)*Z(3*N+3:3:6*N-3)) + barrier_cost + lane_cost+ lane_keep_cost;

end