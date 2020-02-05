function [Z,fval,exitflag] = optimizer_fmincon(xr,Z0,A, B, N ,x_tilde,u_tilde,lb,ub,obstacles)

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
    if Z0==0
       Z0=[Omega*x_tilde;zeros(N*3,1);zeros(N*2,1)]'; 
    end
    % Z = [x_tilde,x;u]
    Mxr=[];
    for i=1:N
        Mxr=[Mxr;eye(3)];
    end
    Ain=[
    ];
    bin=[
        ];

    Aeq=[
         eye(3*N),zeros(3*N),-Gamma
         -eye(3*N),eye(3*N),zeros(N*3,2*N)        
        ];
    beq=[
         Omega*x_tilde
         Mxr*xr         
        ];
    
    % f=V*Z
    fun = @(Z) objective_func(Z,N,obstacles);
    %nonl_con = @(Z) nonlcon(Z,N,uk,obstacles);
    options = optimoptions('fmincon','Display','off'); %,'TolCon',1e-6
    [Z,fval,exitflag] = fmincon(fun,Z0,Ain,bin,Aeq,beq,lb,ub,[],options);
end







