function Z = optimizer_lin(A, B, N ,xf,xk,uL,uU)
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

    % Z = [x;u,t,a]
    Mbelt=zeros(N,N*2);
    for i=1:N
        Mbelt(i,2*i-1:2*i)=ones(1,2);
    end
    p=2*4;
    
    
    Ain=[
         zeros(N,2*N)  ,zeros(N,2*N),eye(N),  -eye(N)
         zeros(N,2*N)  ,zeros(N,2*N),eye(N),  eye(N)
        
        eye(2*N),   zeros(2*N),  zeros(2*N,N), zeros(2*N,N)     %x<xf
        zeros(2*N), eye(2*N),     zeros(2*N,N),  zeros(2*N,N)     %u<10
        zeros(2*N), -eye(2*N),     zeros(2*N,N),  zeros(2*N,N)     %u>0.01
    ];

    Bin=[
         zeros(N,1)
         zeros(N,1)
        
        ones(N*2,1)*xf
        ones(N*2,1)*uU
        ones(2*N, 1)*(-uL)
        ];

    Aeq=[
        Mbelt,  zeros(N,2*N),zeros(N),eye(N)*p
        eye(2*N),-Gamma, zeros(2*N,N),zeros(2*N,N)
        ];
    Beq=[
       ones(N,1)*p
        Omega*xk
        ];
    
    % f=V*Z
    f=[-ones(2*N, 1)*2;ones(2*N, 1);-ones(N,1)*100;zeros(N,1)];
    Z = linprog(f, Ain, Bin,Aeq,Beq);
end





