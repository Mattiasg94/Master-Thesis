function obj_fun = objective_func(Z,MQ,MR,MR_jerk,N,lane_border_min,lanewidth,barrier_weight,center)
Z = Z(:); % Guarantee that Z is a colon vector

%% Initialize costs:
lane_cost = 0;

for i = 0:N-1
    % ---- Easy to read vars ----
    x = Z(3*N+1+3*i);
    y = Z(3*N+2+3*i);
    radius_ego = sqrt((x-center(1))^2 + (y-center(2))^2);

    % ---------------------------
    %% Cost lane_cost
    lane1 = (lane_border_min+(lanewidth/2));
    lane2 = lane1+lanewidth;
%     dist_ymin = 1*(lane1-radius_ego);
%     dist_ymax = 1*(radius_ego-lane2);
    dist_ymin = 12*(lane1-radius_ego);
    dist_ymax = 12*(radius_ego-lane2);

%     lane_cost = lane_cost + 1*dist_ymin*(dist_ymin>=0);
%     lane_cost = lane_cost + 1*dist_ymax*(dist_ymax>=0);
    lane_cost = lane_cost + barrier_weight*(dist_ymin^3+10*dist_ymin)*((barrier_weight*(dist_ymin^3+10*dist_ymin))>=0);
    lane_cost = lane_cost + barrier_weight*(dist_ymax^3+10*dist_ymax)*((barrier_weight*(dist_ymax^3+10*dist_ymax))>=0);

end
Q = MQ(1:3,1:3);
obj_fun = Z(1:(N-1)*3)'*MQ*Z(1:(N-1)*3) + Z(N*6+1:N*8)'*MR*Z(N*6+1:N*8);  % + sum(abs(MR_jerk(end-1:end-1)*Z(3*N+3:3:6*N-3))) + lane_cost;
obj_fun = obj_fun +  50*Z(3*(N-1)+1:N*3)'*Q*Z(3*(N-1)+1:N*3);
end