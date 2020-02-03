function [Z,fval] = nonlinearOpti(A,B,N,xf,Z0,obstacle,umax,umin,options)
%% initial state
Z0_ext = Z0;
num_states = length(A);
num_input = length(B(1,:));

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

%% Standard constraints

Aeq = [ eye(num_states*N)                 -Gamma         ;
        -eye(num_states*N)                Gamma       ];
beq = [  Omega*Z0(1:num_states) ;
         -Omega*Z0(1:num_states)   ];
% Aeq = [];
% beq = [];

%           x                       u                   beta
Ain = [ zeros(num_input*N,num_states*N)           eye(num_input*N)          ;      
        zeros(num_input*N,num_states*N)           -eye(num_input*N)         ;    
%         eye(2*N)                 -Gamma            ;      % Imposes eq constraint
%         -eye(2*N)                Gamma             ;  
        ];

eps = 10^-6;
bin = [ones(num_input*N,1)*(umax);
        ones(num_input*N,1)*(umin);
%     Omega*Z0(1:2);      % Imposes eq constraint
%     -Omega*Z0(1:2);     % Imposes eq constraint
    ];

% lb = [-inf*ones(2*N,1); -inf*ones(2*N,1)];% ; zeros(length(b_bar)*N,1)];
% ub = [inf*ones(2*N,1); inf*ones(2*N,1)];% inf*ones(length(b_bar)*N,1)];
lb = [];
ub = [];

%%  nonlinear prog Z = [x u]
[Z,fval,exitflag] = fmincon(@(Z) AnonymousFunc(Z,obstacle,N,xf) ,Z0_ext,Ain,bin,Aeq,beq,lb,ub,[],options);
% [Z,fval,exitflag] = ga(@(Z) AnonymousFunc(Z,obstacle,N,xf) , 120,Ain,bin,Aeq,beq,lb,ub);

if exitflag == -2
    error('Unable to find a feasible solution. Terminating optimization.')
    % Here we want to impose emergency breaking.
end
% beta = Z(2*N+2*N+1:end);
% beta = b_bar'*Z(2*N+2*N+1:end);
% Z
end

%% Storage, old code snippets:
% Aeq = [eye(2*N)     -Gamma];
% beq = [Omega*Z0(1:2)'];
% Nonlinear constraints:
% radius = 1; @(Z)nonlincon(Z,N,obstacle,radius);







