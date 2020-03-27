function obj_fun = objective_func(Z,MQ,MR,N,obstacles)
Z = Z(:);
obj_fun=Z(1:N*3)'*MQ*Z(1:N*3)+Z(N*6+1:N*8)'*MR*Z(N*6+1:N*8);
end
% 
% Z = Z(:);
% Zr = [];
% ur=[0.2;0];
% xr=[10;5;0];
% 
% for i = 1:N
%     Zr = [Zr; xr];
% end
% for i = 1:N
%     Zr = [Zr; ur];
% end
% 
% obj_fun=(Z(3*N+1:N*6)-Zr(1:3*N))'*MQ*(Z(3*N+1:N*6)-Zr(1:3*N))   +   (Z(6*N+1:8*N)-Zr(3*N+1:end))'*MR*(Z(6*N+1:8*N)-Zr(3*N+1:end));
% end
