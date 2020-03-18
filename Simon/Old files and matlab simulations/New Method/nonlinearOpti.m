function [Z,fval] = nonlinearOpti(A,B,N,xf,Z0,obstacle,ub,lb,options)
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

Aeq = [ eye(num_states*N)                 -Gamma];
beq = [ Omega*Z0(1:num_states)];
Ain = [];
bin = [];

%%  nonlinear prog Z = [x u]
[Z,fval,exitflag] = fmincon(@(Z) AnonymousFunc(Z,obstacle,N,xf) ,Z0_ext,Ain,bin,Aeq,beq,lb,ub,[],options);

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







