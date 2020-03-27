function [cTOT] = nonlcon_snopt(Z,MQ,MR,MR_jerk,N,obstacles,lane_st,ylimit,xk,uk,dt,obstacles_u,r_obs,xr,r_safety_margin) %[cin; ceq]

Z = Z(:);
Qy = MQ(2,2);
 %% FIX DIS
lane_cost = 0;
barrier_cost = 0;
for i = 0:N-1
    %% Go to reference lane constraint
    for j = 1:length(obstacles)
        passed_obs_by3M = (obstacles{j}(1)-Z(3*N+1+3*i) <= -3);
        lane_cost = lane_cost + (passed_obs_by3M==true) * sqrt((lane_st.reference_lane- Z(3*N+2+3*i))^2) *Qy*2;
    end
    
    %% Barrier against the wall
    % Top
    if length(Z) == 1
        disp('hey')
    end
    if  ylimit(1) > Z(3*N+2+3*i) && Z(3*N+2+3*i) > lane_st.lanes(1) % lane= [4 2]
        barrier_cost = barrier_cost + abs((+1000*(Z(3*N+2+3*i)-ylimit(1))) + 1000); %* (ylimit(2)+Z(3*N+2+3*i)>lane(1)) ; %
    end 
    % Bottom 
    if ylimit(end) < Z(3*N+2+3*i) && Z(3*N+2+3*i) < lane_st.lanes(end)
        barrier_cost = barrier_cost + abs((-1000*(Z(3*N+2+3*i)-ylimit(2))) + 1000); %*(Z(3*N+2+3*i)<lane(2)); %
    end 
end
% disp(barrier_cost)
%+ sum(R_jerk(end-1:end-1)*Z(3*N+3:3:6*N-3))
% barrier_cost
obj_fun = Z(1:N*3)'*MQ*Z(1:N*3)+Z(N*6+1:N*8)'*MR*Z(N*6+1:N*8) + sum(MR_jerk(end-1:end-1)*Z(3*N+3:3:6*N-3)) + barrier_cost + lane_cost;



% ceq=zeros(N,1);
uk = uk(:);
xk = xk(:);
for i=1:N
    [A,B] = Linearized_discrete_DD_model(xk,uk,dt);    
    xk1=Z(N*3+3*i-2:N*3+3*i)';
    xk1=xk1(:);
    ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
    xk=xk1;
    uk=Z(N*8+2*i-1:N*8+2*i);
    uk = uk(:);
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
        
%       if  (sqrt((Z(x_iter)-obstacle_comp(1))^2 + (Z(y_iter)-obstacle_comp(2))^2) <= 10 )
        if  (sign(vx_obs) ~= sign(vx) && x < x_obs )... %&& (sqrt((Z(x_iter)-obstacle_comp(1))^2 + (Z(y_iter)-obstacle_comp(2))^2) <= 3 )) ...
            ||  (sign(vx_obs) == sign(vx) && (vx_obs < vx) && x < x_obs) % && (sqrt((Z(x_iter)-obstacle_comp(1))^2 + (Z(y_iter)-obstacle_comp(2))^2) <= 3 )) 
            
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
        else
            cin(iter,1) = 0;
        end
        
        x_iter = x_iter + 3;
        y_iter = y_iter + 3;
        th_iter = th_iter + 3;
        iter = iter + 1;
    end
end

% if (all(cin,'all') == 0) % && isempty(obstacles)
%     cin = zeros(2*N,1);
% end
cTOT = [obj_fun;cin(:);ceq(:)];

