function [Z,fval,exitflag] = optimizer_fmincon(dt,xr,Z0,A, B,MQ,MR,Mxr,Mur,N,x_tilde,u_tilde,xk,lb,ub,obstacles,x0,u0)

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
       Z0=[Omega*x_tilde;zeros(N*3,1);zeros(N*2,1);zeros(N*2,1)]';  
    end
%     Z = [x_tilde,x;u_tilde,u];
    Mxr=[];
    for i=1:N
        Mxr=[Mxr;xr];
    end
    Ain=[
    ];
    bin=[
        ];
    Aeq=[
         eye(3*N),zeros(3*N),-Gamma ,zeros(N*3,2*N)
         -eye(3*N),eye(3*N),zeros(N*3,2*N),zeros(N*3,2*N)
         zeros(2*N,3*N),zeros(2*N,3*N),-eye(2*N),eye(2*N)
%          zeros(3*N),eye(3*N),zeros(N*3,2*N),-Gamma
        ];
    beq=[
         Omega*x_tilde
         Mxr
         Mur 
         %Omega*xk
        ];
%     Aeq=[
%          -eye(3*N),eye(3*N),zeros(N*3,2*N),zeros(N*3,2*N)
%          zeros(2*N,3*N),zeros(2*N,3*N),-eye(2*N),eye(2*N)
%         ];
%     beq=[
%          Mxr
%          Mur 
%         ];


    % f=V*Z
    obj_fun = @(Z) objective_func(Z,MQ,MR,N,obstacles,xr);
%     nonl_con = @(Z) nonlcon(Z,N,obstacles,dt,x0,u0);
%     options = optimoptions('fmincon'); %,'TolCon',1e-6 ,'Display','off' 
%     options.MaxFunctionEvaluations = 3000;
options = [];
    [Z,fval,exitflag] = fmincon(obj_fun,Z0,Ain,bin,Aeq,beq,lb,ub,[],options);
    
end







