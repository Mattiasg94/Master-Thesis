%% Classic horizon detection
% function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles,X_obst)
% ceq=zeros(N,1);
% for i=1:N
%     [A,B] = Linearized_discrete_DD_model(xk,uk,dt);
%     xk1=Z(N*3+3*i-2:N*3+3*i)';
%     ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
%     xk=xk1;
%     uk=Z(N*8+2*i-1:N*8+2*i)';
% end
% obstacles_u={[1;0],[1;0]};
% iter = 0;
% radius = 1.5;
% cin=zeros(length(obstacles)*N,1);
% for j=1:length(obstacles)
%     [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{j},obstacles_u{j},dt);
%     for i = 1:3:3*N
%         iter = iter +1 ;
%         obstacles{j}=A_obstacles*obstacles{j}+B_obstacles*obstacles_u{j};
%         cin(iter,1) = -((Z(3*N+i)-obstacles{j}(1))^2+(Z(3*N+i+1)-obstacles{j}(2))^2) + radius^2;
%
%     end
% end
%

%% Collision cone w/ lambda
% function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles,X_obst,cone_toggle)
% ceq=zeros(N,1);
% for i=1:N
%     [A,B] = Linearized_discrete_DD_model(xk,uk,dt);
%     xk1=Z(N*3+3*i-2:N*3+3*i)';
%     ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
%     xk=xk1;
%     uk=Z(N*8+2*i-1:N*8+2*i)';
% end
% obstacles_u={[1;0],[1;0]};
% iter = 0;
% radius = 1.5;
% k=0;
% % cin=zeros(length(obstacles)*N,1);
% cin = [];
% if cone_toggle == true
% for j=1:length(obstacles)
% %     [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{j},obstacles_u{j},dt);
%     for i = 1:3:3*N
%         iter = iter +2;
%         k = k + 1;
%         [lambda_r,lambda_f,Vec_lambda_r,Vec_lambda_f] = get_collision_cone(Z(3*N+1:3*N+2),X_obst{j}(1:2),radius);
%         slope_f(k) = (Vec_lambda_f(1,1)-Vec_lambda_f(2,1))/(Vec_lambda_f(1,2)-Vec_lambda_f(2,2));
%         X = Z(3*N+1);
%         Y = Z(3*N+2);
%         Vec_lambda_f(:,1) = Vec_lambda_f(:,1) + obstacles_u{j} ;
%         Vec_lambda_f(:,2) = Vec_lambda_f(:,1) + obstacles_u{j} ;
%         position(k) = sign((Vec_lambda_f(2,1) - Vec_lambda_f(1,1)) * (Y - Vec_lambda_f(1,2)) - (Vec_lambda_f(2,2)  - Vec_lambda_f(1,2)) * (X - Vec_lambda_f(1,1)));
%     end
%     position=position(:);
%     cin =[cin; position + 1];
% end
% cin = cin(:);
% end


%% Collision cone
function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles,cone_toggle)
ceq=zeros(N,1);
cin = [];
for i=1:N
    [A,B] = Linearized_discrete_DD_model(xk,uk,dt);
    xk1=Z(N*3+3*i-2:N*3+3*i)';
    ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
    xk=xk1;
    uk=Z(N*8+2*i-1:N*8+2*i)';
end
obstacles_u={[1;0],[1;0]};
iter = 0;
radius = 1; % distance between ego and obst

r = [];
v = [];
for j=1:length(obstacles)
    r = [];
    v = [];
    iter = 0;
    for i = 1:3:3*N-3
        iter = iter +1 ;
        [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{j}(:,iter),obstacles_u{j},dt);
        X_obst_curr = obstacles{j}(:,iter);
        X_obst_next = (A_obstacles*X_obst_curr+B_obstacles*obstacles_u{1}); 
        delta_x = Z(3*N+i+3)-Z(3*N+i);
        delta_y = Z(3*N+i+4)-Z(3*N+i+1);
        delta_x_obst = X_obst_next(1)-X_obst_curr(1);
        delta_y_obst = X_obst_next(2)-X_obst_curr(2);

        r = [r; Z(3*N+i) - X_obst_curr(1);
            Z(3*N+i+1) - X_obst_curr(2)];
        v = [v; uk(1) - obstacles_u{j}(1);
            uk(2) - obstacles_u{j}(2)];
        cin(iter,j) =  radius^2 + dot(r,v)^2/norm(v)^2 - norm(r)^2;
        obstacles{j}(:,iter+1) = X_obst_next;
    end
    
    
end
% if all((cin) <= R^2 + dot(r,v)^2/norm(v)^2 - norm(r)^2)
%     disp('PROGRESS!')
% end
if length(cin(1,:))>=2
    cin =[cin(:,1); cin(:,2)];
else
    cin =[cin(:,1)];
end

