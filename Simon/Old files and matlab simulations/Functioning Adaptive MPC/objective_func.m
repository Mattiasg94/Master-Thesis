function obj_fun = objective_func(Z,MQ,MR,N,obstacles,xf)

obj_fun=Z(1:N*3)*MQ*Z(1:N*3)'+Z(N*6+1:N*8)*MR*Z(N*6+1:N*8)';
% obj_fun = 0;
% for i = 1:3:2*N % OBS!! i = 1 bör ge 2 st f
%     obj_fun= obj_fun + (Z(3*N+i)-xf(1))^2 + (Z(3*N+i+1)-xf(2))^2; % + 1/(abs((x(i)-obstacle(1)))^2 + (x(i+1)-obstacle(2))^2);
% end

end

