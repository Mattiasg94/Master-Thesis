function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs,xr,r_safety_margin,dist_cond,lines_st,lanewidth,lane_offset_x,lane_offset_y,lane_border_min,lane_border_max,road_radius,obstacles_lanes,plot_x_curv,plot_y_curv,obstacle_radiusa,center)
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
obstacles_save = [];
r_tot = r_obs + r_safety_margin;
cone_bool = true;
x_collision = [];
y_collision = [];
for j=1:length(obstacles)
    x_iter = 3*N+1;
    y_iter = 3*N+2;
    v_iter = 8*N+1;
    th_iter = 3*N+3;
    obstacle_comp = obstacles{j}(:);
    obstacle_comp_p1 = obstacle_comp; % initialize obstacle_comp
    for i = 1:N
        
        %% EASY TO READ VARIABLE UPDATE
        u = Z(v_iter);
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
        
        ego_dist = sqrt((x-xr(1))^2+(y-xr(2))^2);
        obs_dist = sqrt((x_obs-xr(1))^2+(y_obs-xr(2))^2);   
        r_circ_ego = sqrt( (x - center(1))^2 + (y-center(2))^2 );
        r_circ_obs = sqrt( (x_obs - center(1))^2 + (y_obs-center(2))^2 );       
        
        if  obs_dist<ego_dist && sqrt((x_obs-x)^2 - (y_obs-y)^2)<dist_cond
            v_tan = get_tang_v_ego(u,x,y,th,center); 
            [t_impact,~]=get_intersection_time(x,y,u,x_obs,y_obs,v_obs,r_circ_ego,r_circ_obs,center);
            [x_impact, y_impact, ~, ~] = obs_move_line(t_impact, lines_st.lanes(j), v_obs, x_obs, y_obs,center,road_radius,lanewidth);

            %% Formulas:
            vx_obs=0; % Hanteras d� den projas fram.
            vy_obs=0;
            
            v_obs_x = 0;
            v_obs_y = 0;
            r_vec = [x-x_impact;y-y_impact];
            vab = [cos(th)*u-v_obs_x; sin(th)*u-v_obs_y];
            rterm = (norm(r_vec))^2;
            lterm = (norm(vab))^2; 
            uterm = (dot(r_vec,vab))^2;
            cone = (r_tot^2)*lterm-(rterm*lterm-uterm);
            
%             rterm = sqrt((x-x_impact)^2+(y-y_impact)^2);
%             uterm = ((cos(th)*u-vx_obs)*(x-x_obs) + (sin(th)*u-vy_obs)*(y-y_impact))^2;
%             lterm = sqrt((cos(th)*u-vx_obs)^2+(sin(th)*u-vy_obs)^2);
%             cone = r_tot^2*lterm^2 - rterm^2*lterm^2 + uterm;
            
            cin(iter,1) = cone;
        end
        
        %% Update iters:
        x_iter = x_iter + 3;
        y_iter = y_iter + 3;
        th_iter = th_iter + 3;
        v_iter = v_iter + 2;
        iter = iter + 1;
    end
end
% cin = [cin; r_circ_ego-lane_border_min; lane_border_max-r_circ_ego ];
end