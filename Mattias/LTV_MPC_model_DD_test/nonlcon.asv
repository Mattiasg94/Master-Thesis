function [cin,ceq] = nonlcon(Z,N,xk,uk)
ceq=[];
cin=0;
for i=1:N
    [A,B] = Linearized_discrete_DD_model(xk,uk,0.5); 
    
    xk1=Z(N*3+3*i-2:N*3+3*i)';
    ceq(3*i-2:3*i,1)=xk1-A*xk-B*uk;
    xk=Z(N*3+3*i-2:N*3+3*i)';
    uk=Z(N*6+2*i-1:N*6+2*i)';

end


    
    
