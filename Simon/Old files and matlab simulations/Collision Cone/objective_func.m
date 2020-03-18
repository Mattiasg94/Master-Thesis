function obj_fun = objective_func(Z,MQ,MR,N,obstacles,r_obs)
Z = Z(:);
% repulsive_region = 0;
% obs_w_offset = {};
%  1/(abs((x(i)-obstacle(1)))^2
% for j = 1:length(obstacles)
%     obs_w_offset{j} = obstacles{j}(1:2)+r_obs*[cos(obstacles{j}(3)); sin(obstacles{j}(3))];
%     for i = 1:3:3*N % OBS!! i = 1 bör ge 2 st f
%         repulsive_region = repulsive_region  +  1/((Z(3*N+i)-obs_w_offset{j}(1))^2 + (Z(3*N+(i+1))-obs_w_offset{j}(2))^2);
%     end
% end

obj_fun=Z(1:N*3)'*MQ*Z(1:N*3)  +  Z(N*6+1:N*8)'*MR*Z(N*6+1:N*8);  %+  repulsive_region;



end
