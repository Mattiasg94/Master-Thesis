function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs)
ceq=zeros(N,1);
for i=1:N
    [A,B] = Linearized_discrete_DD_model(xk,uk,dt);
    xk1=Z(N*3+3*i-2:N*3+3*i)';
    ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
    xk=xk1;
    uk=Z(N*8+2*i-1:N*8+2*i)';
end

iter = 1;
cin=zeros(length(obstacles)*N,1);
for j=1:length(obstacles)
    [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{j},obstacles_u{j},dt);
    x_iter = 3*N+1;
    y_iter = 3*N+2;
    th_iter = 3*N+3;
    for i = 1:N
        %% Distance cond.:  
        if sqrt((Z(x_iter)-obstacles{j}(1))^2+(Z(y_iter)-obstacles{j}(2))^2) < 2
        %% Obstacle is faster cond.: 
%         if Z(8*N+1+2*(i-1)) =< obstacles{j}
        %% Mix
%         if  Z(8*N+1+2*(i-1)) =< obstacles_u{j} ||  sqrt((Z(x_iter)-obstacles{j}(1))^2+(Z(y_iter)-obstacles{j}(2))^2) < 2
            obstacles{j} = A_obstacles*obstacles{j}+B_obstacles*obstacles_u{j};
            r = [Z(x_iter);Z(y_iter)]-obstacles{j}(1:2);
            vab = Z(8*N+1+(2*(i-1)))*[cos(Z(th_iter)); sin(Z(th_iter))] - [obstacles_u{j}(1);obstacles_u{j}(2)];
            d_square=-norm(r)^2+dot(r,vab)^2/norm(vab)^2;
            cin(iter,1) = d_square+(r_obs)^2;
          
        else
            cin(iter,1) = 0;
        end
        
        iter = iter + 1;
        x_iter = x_iter + 3;
        y_iter = y_iter + 3;
        th_iter = th_iter + 3;
    end
    
end




