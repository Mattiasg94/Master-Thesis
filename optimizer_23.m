function Z = optimizer_23(A, B, N, TUb, TLb, T0, d, rho, kappa)
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
    
    % Z = [u;epsLb;epsUb;t;tInf]
    Mu = [eye(N), zeros(N, 3*N + 1)]; % u = Mu*Z and so on
    MepsLb = [zeros(N), eye(N), zeros(N, 2*N + 1)];
    MepsUb = [zeros(N, 2*N), eye(N), zeros(N, N +1)];
    Mt = [zeros(N, 3*N), eye(N), zeros(N, 1)];
    MtInf = [zeros(1,4*N), 1];
    test=ones(N,1)*MtInf;
    Ain = [Mu - Mt
        -Mu - Mt
        kappa*Mu - ones(N,1)*MtInf
        -kappa*Mu - ones(N,1)*MtInf
        -MepsLb - Gamma*Mu
        Gamma*Mu - MepsUb
        -MepsLb
        -MepsUb];
    bin = [zeros(4*N, 1)
        Omega*T0 - TLb*ones(N, 1) + Gamma/B*d
        TUb*ones(N, 1) - Omega*T0 - Gamma/B*d
        zeros(2*N, 1)];
    f = (ones(N, 1)'*Mt + MtInf + rho*(ones(N, 1)'*MepsUb + ones(N, 1)'*MepsLb))';
    
    Z = linprog(f, Ain, bin);
    
    
end