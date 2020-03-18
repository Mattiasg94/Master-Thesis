function [cin,ceq]  = nonlincon(Z,N,obstacle,radius) 
cin = 0;
iter = 0;
for i = 1:2:2*N
    iter = iter +1 ;
    cin(iter) = ((Z(i)-obstacle(1))^2+(Z(i+1)-obstacle(2))^2 - radius^2);
end
ceq = [];
end




