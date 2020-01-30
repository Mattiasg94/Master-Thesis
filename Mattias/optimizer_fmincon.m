function Z = optimizer_fmincon(A, B, N ,xf,xk,uL,uU)
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

    % Z = [x;u]
    Mbelt=zeros(N,N*2);
    for i=1:N
        Mbelt(i,2*i-1:2*i)=ones(1,2);
    end
    p=2*4;
    
    
    Ain=[        
        eye(2*N),   zeros(2*N)     %x<xf
        zeros(2*N), eye(2*N)       %u<10
        zeros(2*N), -eye(2*N)      %u>0.01
    ];

    bin=[
        ones(N*2,1)*xf
        ones(N*2,1)*uU
        ones(2*N, 1)*(-uL)
        ];

    Aeq=[
        eye(2*N),-Gamma
        ];
    beq=[
        Omega*xk
        ];
    
    % f=V*Z
    %f=[-ones(2*N, 1)*2;ones(2*N, 1)];
    fun = @(Z)-(Z(1)+Z(2)+);
    Z0=[xk,zeros(N*2,1)];
    Z = fmincon(fun,Z0,Ain,bin,Aeq,beq);
end





