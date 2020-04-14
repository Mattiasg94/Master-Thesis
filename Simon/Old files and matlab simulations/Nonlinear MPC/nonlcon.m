function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs,xr,r_safety_margin,dist_cond,lanewidth,road_radius,obstacles_lanes,center)
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
cin = zeros(length(obstacles)*N,1);
r_tot = r_obs + r_safety_margin;

for j=1:length(obstacles)
    x_iter = 3*N+1;
    y_iter = 3*N+2;
    th_iter = 3*N+3;
    v_iter = 8*N+1;
    
    for i = 1:N
        
        %% EASY TO READ VARIABLE UPDATE   
        x = Z(x_iter);
        y = Z(y_iter);
        th = Z(th_iter);
        v = Z(v_iter);
        x_obs = obstacles{j}(1);
        y_obs = obstacles{j}(2);
        th_obs = obstacles{j}(3);
        v_obs = obstacles_u{j}(1); % Constant
        w_obs = obstacles_u{j}(2); % Constant
        
        ego_dist = sqrt((x-xr(1))^2+(y-xr(2))^2);
        obs_dist = sqrt((x_obs-xr(1))^2+(y_obs-xr(2))^2);   
        r_circ_ego = sqrt(  (x-center(1))^2 + (y-center(2))^2  );
        r_circ_obs = sqrt(  (x_obs-center(1))^2 + (y_obs-center(2))^2  );

        if (ego_dist>=obs_dist) && ( sqrt((x_obs-x)^2+(y_obs-y)^2) <= dist_cond )
            v_tan = get_tang_v_ego(v,x,y,th,center);  
            [t_impact,~] = get_intersection_time(x,y,v_tan,x_obs,y_obs,v_obs,r_circ_ego,r_circ_obs,center);
            [x_impact, y_impact, ~, ~] = obs_move_line(t_impact, obstacles_lanes{j}, v_obs, x_obs, y_obs,center,road_radius,lanewidth);

            %% Formulas:            
            v_obs_x = 0;
            v_obs_y = 0;
            r_vec = [x-x_impact; y-y_impact];
            vab = [cos(th)*v-v_obs_x; 
                   sin(th)*v-v_obs_y];
            rterm = (norm(r_vec))^2;
            lterm = (norm(vab))^2; 
            uterm = (dot(r_vec,vab))^2;
            cone = (r_tot^2)*lterm-(rterm*lterm-uterm);
            cin(iter,1) = cone;
        else
            cin(iter,1) = 0;
        end
        
        %% Update iters:
        x_iter = x_iter + 3;
        y_iter = y_iter + 3;
        th_iter = th_iter + 3;
        v_iter = v_iter + 2;
        iter = iter + 1;
    end
    
end
% cin = [cin; r_circ_ego-lane_border_min; lane_border_max-r_circ_ego ]; % Försök med hårda constraint på border


end