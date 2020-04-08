function obj_fun = objective_func(dt,Z,MQ,MR,MR_jerk,N,obstacles,obstacles_u,lines_st,x_offset,y_offset,lane_border_max,lane_border_min,xr,lanewidth,dist_cond,obstacle_radius,impact)
Z = Z(:); % Guarantee that Z is a colon vector

%% Initialize costs:
lane_cost = 0;
lane_keep_cost = 0;
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
    lane_cost = lane_cost + (dist_ymin^3+10*dist_ymin)*(dist_ymin^3+10*dist_ymin>=0);
    lane_cost = lane_cost + (dist_ymax^3+10*dist_ymax)*(dist_ymax^3+10*dist_ymax>=0);
    
    
%     lane_keep_cost = lane_keep_cost + 500*(radius_ego - lines_st.reference_lane)^2;
    
end
obj_fun = Z(1:N*3)'*MQ*Z(1:N*3)+Z(N*6+1:N*8)'*MR*Z(N*6+1:N*8) + sum(MR_jerk(end-1:end-1)*Z(3*N+3:3:6*N-3)) + lane_cost+ lane_keep_cost;

end