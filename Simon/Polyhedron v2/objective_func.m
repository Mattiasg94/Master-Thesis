function obj_fun = objective_func(Z,MQ,MR,N,obstacles,Zr)
% Z = Z(:);
% obj_fun=Z(1:N*3)'*MQ*Z(1:N*3)+Z(N*6+1:N*8)'*MR*Z(N*6+1:N*8);
% end

Z = Z(:);
obj_fun=(Z(1:3*N)-Zr(1:3*N))'*MQ*(Z(1:3*N)-Zr(1:3*N))   +   (Z(3*N+1:end)-Zr(3*N+1:end))'*MR*(Z(3*N+1:end)-Zr(3*N+1:end));
end
