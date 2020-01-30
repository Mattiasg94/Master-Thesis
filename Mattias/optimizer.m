function Z = optimizer(A, B, N ,xf,xk,uL,uU)
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

    % Z = [x;u,t]    
    Ain=[
        eye(2*N),    zeros(2*N)       %x<xf
        zeros(2*N), eye(2*N)        %u<10
        zeros(2*N),  -eye(2*N)         %u>0.01
    ];

    Bin=[
        ones(N*2,1)*xf
        ones(N*2,1)*uU
        ones(2*N, 1)*(-uL)
        ];

    Aeq=[
        eye(2*N),-Gamma
        ];
    Beq=[
        Omega*xk
        ];
    
    % f=V*Z
    f=[-ones(2*N, 1)*5;ones(2*N, 1)];
%     f=[-8;-8;ones(2*N, 1)];
%     H=[2 1 0 0
%        1 2 0 0
%        0 0 0 0
%        0 0 0 0];
%     Z= quadprog(H,f, Ain, Bin,Aeq,Beq);
    Z = linprog(f, Ain, Bin,Aeq,Beq);
end


