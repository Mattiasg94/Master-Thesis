function [Z,fval] = simpleOpti(A,B,N,xf,x0,obstacle)

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

%% Quadprog
% WORKING:
% [x u]

Aeq = [eye(2*N)         -Gamma];
beq = [Omega*x0'];

Ain = [zeros(2*N,2*N)           eye(2*N);
       zeros(2*N,2*N)           -eye(2*N) ];
bin = [ones(2*N,1)*5;
    ones(2*N,1)*5;
    ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BARRIER TEST! Approx min 1/x as min -x
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aeq = [eye(2*N)         -Gamma];
% beq = [Omega*x0'];
% 
% Ain = [zeros(2*N,2*N)           eye(2*N);
%     zeros(2*N,2*N)           -eye(2*N);
%     % Test barrier function
%     eye(2*N)                 zeros(2*N,2*N);
%     -eye(2*N)                 zeros(2*N,2*N);
%     ];
% 
% barrier1 = ((-1).^(0:2*N-1))==1;
% barrier1=barrier1*4;
% barrier1(barrier1==0) = 2;
% 
% barrier2 = ((-1).^(0:2*N-1))==1;
% barrier2=barrier2*-3;
% barrier2(barrier2==0) = -1;
% 
% bin = [ones(2*N,1)*5;
%     ones(2*N,1)*5;
%     % Test barrier function
%     barrier1(:)
%     barrier2(:)
%     ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

H = zeros(4*N,4*N);
f = zeros(4*N,1);

for i = 1:length(H)/2   % ONLY WRITE THIS ON X, NOT U!
    for j = 1:length(H)/2
        H(i,j) = 2*(i==j)*(mod(i,2)==1) + 2*(i==j)*(mod(i,2)==0); % x in X AND y in X
    end
    f(i) = (-2*xf(1))*(mod(i,2)==1)+ ((-2*xf(2))*(mod(i,2)==0)); % x in X, y in X
end

[Z,fval,exitflag] = quadprog(H,f,Ain,bin,Aeq,beq,[],[],x0);




end



