function obj_fun = objective_func(Z,MQ,MR,MR_jerk,N,obstacles,lane_st,ylimit)
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

end