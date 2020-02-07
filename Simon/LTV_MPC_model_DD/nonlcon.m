function [cin,ceq] = nonlcon(Z,N,xk,uk,dt,obstacles)
ceq=[];
cin=0;
for i=1:N
    [A,B] = Linearized_discrete_DD_model(xk,uk,dt);    
    xk1=Z(N*3+3*i-2:N*3+3*i)';
    ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
    xk=xk1;
    uk=Z(N*8+2*i-1:N*8+2*i)';
end

iter = 0;
radius = 1;
for i = 1:3:3*N
    iter = iter +1 ;
    cin(iter,1) = -((Z(3*N+i)-obstacles(1))^2+(Z(3*N+i+1)-obstacles(2))^2) + radius^2;
%     3*N+i
%     3*N+i+1
end
 
 
    
    



    
    
