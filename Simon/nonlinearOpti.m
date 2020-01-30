function [Z,fval] = nonlinearOpti(A,B,N,xf,x0,obstacle,umax,umin,options)

Omega = A;
for k = 2:N
    Omega = [Omega;A^k];
end

Gamma = [];
for i = 1:N
    row = [];
    for j = 1:N
        row = [row, A^abs(i-j)*B*(j<=i)];
    end
    Gamma = [Gamma; row];
end

x0_ext = [];
for i = 1:N
    x0_ext = [x0_ext; x0(1:2)'];
end 
for i = 1:N
    x0_ext = [x0_ext; x0(3:4)'];
end 

%% nonlinear prog
% [x u]
% Aeq = [eye(2*N)     -Gamma];
% beq = [Omega*x0(1:2)'];
Aeq = [];
beq = [];

Ain = [zeros(2*N,2*N)           eye(2*N);
       zeros(2*N,2*N)           -eye(2*N);
       eye(2*N)                 -Gamma
       -eye(2*N)                Gamma
       ];
bin = [ones(2*N,1)*(umax);
       ones(2*N,1)*(umin);
       Omega*x0(1:2)';
       -Omega*x0(1:2)'];

% Aeq = [];
% beq = [];
% Ain = [];
% bin = [];

%%  Tune Objective function? 
% x^2+y^2-2*10*x-2*0*y + 1/(abs((x-3)^2 + (y-1)^2)) 
% ^ Nice func for obstacle

% Distance minimizers:
% x(1)^2+x(2)^2 -2*10*x(1)-2*0*x(2)
% (x^2-10))^2 + (y^2-0)^2 + 1/(abs((x-3))^2 + (y-1)^2))
% Med tilt:
% abs(x^2-10)) + abs(y^2-0) -10x + 1/(abs((x-3))^2 + (y-1)^2))

% (x-10))^2 + (y-0)^2 + 1/(abs((x-3))^2 + (y-1)^2))
% (x(1)-xf(1)))^2 + (x(2)-xf(2))^2 + 1/(abs((x(1)-obstacle(1,1)))^2 + (x(2)-obstacle(1,2))^2))
% (x(1)-xf(1))^2+(x(2)-xf(2))^2

[Z,fval,exitflag] = fmincon(@(x) AnonymousFunc(x,[],N,xf),x0_ext,Ain,bin,Aeq,beq,[],[],[],options);
% fmincon(@(x) (x(1)-xf(1))^2+(x(2)-xf(2))^2   ,x0_ext,Ain,bin,Aeq,beq,[],[],[],options);
% + 1/(abs((x(1)-obstacle(1,1)))^2 + (x(2)-obstacle(1,2))^2)

end







