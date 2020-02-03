function [Z,fval] = nonlinearOpti(A,B,N,xf,Z0,obstacle,umax,umin,options)
%% initial state
Z0_ext = Z0;

%% Build matrices Z = [x u]
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

%%  POLYHEDRON CONSTRAINTS:
% Currently only one body
theta = 0;
A_bar = [sin(theta) -cos(theta);
        -sin(theta) cos(theta);
         cos(theta) sin(theta);
        -cos(theta) -sin(theta);
         sin(theta) -cos(theta);
        -sin(theta) cos(theta);
         cos(theta) sin(theta);
        -cos(theta) -sin(theta)];
   
    
b_bar = [
    [1/2;
    1/2;
    1/2;
    1/2] + A_bar(1:4,1:2)*Z0_ext(1:2); % Ego
    [1/2;
    1/2;
    1/2;
    1/2] + A_bar(5:8,1:2)*[obstacle(1); obstacle(2)];  % Hinder
    ];

%% Standard constraints
% Mb_bar=[];
MA_bar=[];
for i=1:N
    MA_bar = blkdiag(MA_bar,A_bar);
%     Mb_bar = blkdiag(Mb_bar,b_bar);
end




%           x                                    u                                  beta
Aeq = [ eye(2*N)                                -Gamma                              zeros(2*N,N*length(b_bar))
        zeros(length(MA_bar(1,:)),2*N)          zeros(length(MA_bar(1,:)),2*N)      MA_bar'];
beq = [ Omega*Z0(1:2)
        zeros(length(MA_bar(1,:)),1)];

%           x                       u                   beta
Ain = [ zeros(2*N,2*N)           eye(2*N)                 zeros(2*N,8*N);
        zeros(2*N,2*N)           -eye(2*N)                zeros(2*N,8*N);
        zeros(8*N,2*N)           zeros(8*N,2*N)           -eye(8*N);
        zeros(N,2*N)             zeros(N,2*N)             Mb_bar'];

eps = 10^-6;
bin = [ ones(2*N,1)*(umax);
        ones(2*N,1)*(umin);
        zeros(8*N,1);
        -eps*ones(N,1)];

% lb = [-inf*ones(2*N,1); -inf*ones(2*N,1); zeros(length(b_bar)*N,1)];
% ub = [inf*ones(2*N,1); inf*ones(2*N,1); inf*ones(length(b_bar)*N,1)];
lb = [];
ub = [];
%%  nonlinear prog Z = [x u]
[Z,fval,exitflag] = fmincon(@(Z) AnonymousFunc(Z,obstacle,N,xf) ,Z0_ext,Ain,bin,Aeq,beq,lb,ub,[],options);
% [Z,fval,exitflag] = ga(@(Z) AnonymousFunc(Z,obstacle,N,xf) , 120,Ain,bin,Aeq,beq,lb,ub);

if exitflag == -2
    error('Unable to find a feasible solution. Terminating optimization.')
    % Here we want to impose emergency breaking.
end


end


%% Storage, old code snippets:
% Aeq = [eye(2*N)     -Gamma];
% beq = [Omega*Z0(1:2)'];
% Nonlinear constraints:
% radius = 1; @(Z)nonlincon(Z,N,obstacle,radius);







