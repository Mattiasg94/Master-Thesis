function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs)
ceq = []; %zeros(N,1);
cin = [];

for i=1:N
    [A,B] = Linearized_discrete_DD_model(xk,uk,dt);    
    xk1=Z(3*i-2:3*i)';
    ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
    xk=xk1;
    uk=Z(N*3+2*i-1:N*3+2*i)';
end

% iter = 1;
% cin=zeros(length(obstacles)*N,1);
% for j=1:length(obstacles)
%     [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{j},obstacles_u{j},dt);
%     th_iter = 3;
%     for i = 1:N          
%         obstacles{j} = A_obstacles*obstacles{j}+B_obstacles*obstacles_u{j};
%         r = [Z(3*N+1+(3*(i-1)));Z(3*N+2+(3*(i-1)))]-obstacles{j}(1:2);
%         vab = Z(8*N+1+(2*(i-1)))*[cos(Z(3*N+th_iter)); sin(Z(3*N+th_iter))] - [obstacles_u{j}(1);obstacles_u{j}(2)];   
%         d_square=-norm(r)^2+(dot(r,vab))^2/norm(vab)^2;
%         cin(iter,1) = d_square+(r_obs)^4;

%         r=[Z(3*N+1+(3*(i-1)));Z(3*N+2+(3*(i-1)))]-obstacles{j}(1:2);
%         vab=Z(8*N+1+(2*(i-1)))*[cos(Z(3*N+th_iter)); sin(Z(3*N+th_iter))]-[obstacles_u{j}(1);obstacles_u{j}(2)];
%         d=sqrt(norm(r)^2-(dot(r,vab))^2/norm(vab)^2);
%         cin(iter,1) = r_obs-d;
% 
%         iter = iter + 1;
%         th_iter = th_iter + 3;
%     end
    
end   




