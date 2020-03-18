function [cin,ceq] = nonlcon(Z,N,obstacles,dt,x0,u0)
cin=0;
ceq=[];
min_dist=1; % low=close to
%% Equality constraint w/ step varying A, B
% iter = 4;
% [A,B] = Linearized_discrete_DD_model(x0,u0,dt);
% ceq = Z(4  : 4+2)'-A*x0-B*u0; 
% for i=4:10:10*(N-1)
%     xk=Z(i  : i+2)';
%     uk=Z(i+5  : i+5+1)';
%     [A,B] = Linearized_discrete_DD_model(xk,uk,dt);  
%     xk1=Z(i+10:i+10+2)';
%     ceq(iter:iter+2)=xk1-A*xk-B*uk;
%     iter = iter + 3;
% end






%% Obstacle
% if any(obstacles)~=0
%     j = 1;
%     for i = 3*N+1:3:6*N     % (Num states+ num inputs) *N
%         
%         obs_dist_const(j:j+1) = [-sqrt((Z(i)-obstacles(1))^2 + (Z(i+1)-obstacles(2))^2) + min_dist;
%                                  -sqrt((Z(i)-obstacles(3))^2 + (Z(i+1)-obstacles(4))^2) + min_dist];
%         j = j+2;
%     end
%     
%     obs_dist_const = obs_dist_const(:);
%     cin = obs_dist_const;
% end

%% Velocity 
% for i=1:N-1
%    cin(i,1)=abs(Z(i*10)-Z((i+1)*10))-1;
% end
% 
% cin = [cin;obs_dist_const];
