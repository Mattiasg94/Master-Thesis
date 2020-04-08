function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs,xr,r_safety_margin)
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
r_tot= r_obs + r_safety_margin;
for j=1:length(obstacles)
    [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{j},obstacles_u{j},dt);
    x_iter = 3*N+1;
    y_iter = 3*N+2;
    th_iter = 3*N+3;
    obstacle_comp = obstacles{j};
    
    for i = 1:N
        %% EASY TO READ VARIABLES UPDATE
        u = Z(8*N+1+(2*(i-1)));
        x = Z(x_iter);
        y = Z(y_iter);
        th = Z(th_iter);
        x_obs = obstacle_comp(1);
        vx = sin(th)*u;
        vy = cos(th)*u;
        y_obs = obstacle_comp(2);
        vx_obs = obstacles_u{j}(1)*cos(obstacle_comp(3));
        vy_obs = obstacles_u{j}(2)*sin(obstacle_comp(3));
        
        %% Conditions:
        ego_dist = sqrt((x-xr(1))^2+(y-xr(2))^2);
        obs_dist = sqrt((x_obs-xr(1))^2+(y_obs-xr(2))^2);
        passed_obs = (obs_dist-ego_dist > 0);
        obs_faster_than_ego = u < v_obs;
        obs_driving_towards = norm(th - th_obs)>=pi/2;
%         skip_due_to_dir_and_vel = (obs_driving_towards) : (@() false ) : (@()(obs_faster_than_ego: @()true :@()false)); 
        if obs_driving_towards
            skip_due_to_dir_and_vel = false;
        else
            if ~obs_driving_towards
                skip_due_to_dir_and_vel = true;
            else
                skip_due_to_dir_and_vel = false;
            end
        end
        % deactivate_activate_cone = passed_obs:@()true:@()(skip_due_to_dir_and_vel:@() true:@() false);
        if passed_obs
            deactivate_activate_cone = true;
        else
            if skip_due_to_dir_and_vel
                deactivate_activate_cone = true;
            else
                deactivate_activate_cone = false;
            end
        end
        
        
        if  ~deactivate_activate_cone
            obstacle_comp = A_obstacles*obstacles{j}+B_obstacles*obstacles_u{j};
            %% Formulas:
            rterm = (x-x_obs)^2+(y-y_obs)^2; %OK
            uterm = ((cos(th)*u-vx_obs)*(x-x_obs) + (sin(th)*u-vy_obs)*(y-y_obs))^2;
            lterm = (cos(th)*u-vx_obs)^2+(sin(th)*u-vy_obs)^2; % Dividerar med
            cone = r_tot^2*lterm - rterm*lterm + uterm; %-(rterm-r_tot^2 - uterm/lterm);
            %% Check if traj or ref point
            if length(xr) > 3 %% ONLY FOR TRAJ!   % Use boolean instead.
                ego_dist = norm([Z(x_iter) Z(y_iter)]-[xr(1,i); xr(2,i)]);
                obst_dist = norm([obstacles{j}(1) obstacles{j}(2)]-[xr(1,i); xr(2,i)]);
            else    %% ONLY FOR REF POINT!
                ego_dist = sqrt((x-xr(1))^2 + (y-xr(2))^2);
                obst_dist = sqrt((x_obs-xr(1))^2 + (y_obs-xr(2))^2);
            end
            
            %% Apply constraint if within condition:
%             if ego_dist >= obst_dist  % <- Distance guard!
                cin(iter,1) = cone; %((r_tot)^2)*norm(vab)^2-((norm(r)^2*norm(vab)^2)-dot(r,vab)^2);
%             end
        end
        
        x_iter = x_iter + 3;
        y_iter = y_iter + 3;
        th_iter = th_iter + 3;
        iter = iter + 1;
    end
end