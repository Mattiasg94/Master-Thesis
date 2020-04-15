function [Z,fval,exitflag,timerVal] = optimizer_fmincon(xk,uk,dt,dv,dw,Z0,...
                MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta,N,lb,ub,obstacles,...
                obstacles_u,r_obs,xr,MR_jerk,r_safety_margin,...
                lane_border_min,lanewidth,...
                dist_cond,road_radius,obstacles_lanes,warmstart,center,barrier_weight,Qt,options)

if (all(Z0==0) || (~warmstart))
    Z0=[zeros(N*3,1);zeros(N*3,1);zeros(N*2,1);zeros(N*2,1)]';
end
% Z = [x_tilde,x;u_tilde,u]

Ain=[
    %Delta v
    zeros(1,N*3),zeros(1,N*3),zeros(1,N*2),[1,zeros(1,N*2-1)]
    zeros(N-1,N*3),zeros(N-1,N*3),zeros(N-1,N*2),Mu1_delta
    zeros(N-1,N*3),zeros(N-1,N*3),zeros(N-1,N*2),-Mu1_delta
    %Delta
    zeros(1,N*3),zeros(1,N*3),zeros(1,N*2),[0,1,zeros(1,N*2-2)]
    zeros(N-1,N*3),zeros(N-1,N*3),zeros(N-1,N*2),Mu2_delta
    zeros(N-1,N*3),zeros(N-1,N*3),zeros(N-1,N*2),-Mu2_delta
    ];
bin=[
    %Delta v
    uk(1)+dv
    ones(2*(N-1),1)*dv
    %Delta w
    uk(2)+dw
    ones(2*(N-1),1)*dw
    ];

% Error relation
Aeq=[
    -eye(3*N),eye(3*N),zeros(N*3,2*N),zeros(N*3,2*N)
    zeros(2*N,3*N),zeros(2*N,3*N),-eye(2*N),eye(2*N)
    ];
beq=[
    Mxr
    Mur
    ];

%% FMINCON - Interior, sqp, active-set
obj_fun = @(Z) objective_func(Z,MQ,MR,MR_jerk,N,lane_border_min,lanewidth,barrier_weight,center,Qt);
nonl_con = @(Z) nonlcon(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs,xr,r_safety_margin,dist_cond,lanewidth,road_radius,obstacles_lanes,center);

tic
% [Z,fval,exitflag] = fmincon(obj_fun,Z0,[],[],[],[],lb,ub,nonl_con,options);
[Z,fval,exitflag] = fmincon(obj_fun,Z0,Ain,bin,Aeq,beq,lb,ub,nonl_con,options);
timerVal = toc;


%% ALLA ANDRA OPTIMIZERS, EJ FUNGERANDE

%% YALMIP - IPOPT
% % SNOPT - An augmented Lagrangian merit function ensures convergence from an arbitrary point. Infeasible problems are treated methodically via elastic bounds on the nonlinear constraints. SNOPT allows the nonlinear constraints to be violated (if necessary) and minimizes the sum of such violations.
% % NOMAD - Kräven nonlinear ineq constraints och bounded problems.
% % LBFGSB Funkar
% obj_fun = @(Z) objective_func(Z,MQ,MR,MR_jerk,N,obstacles,lane_st,ylimit);
% opts = optiset('solver','ipopt', 'maxiter',3000,'maxfeval',3000,'maxtime',0.5);
% nonl_con = @(Z) nonl_con_ipopt(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs,xr,r_safety_margin); 
% %       cin, 20x1          ceq, 30x1
% cl = [ ones((length(obstacles))*N,1)*-inf; zeros(3*N,1) ]; %
% cu = [ ones((length(obstacles))*N,1)*0; zeros(3*N,1) ];
% 
% %   Build OPTI Problem
% Opt = opti('fun',obj_fun,'eq',Aeq,beq,'nl',nonl_con,cl,cu,'ineq',Ain,bin,'bounds',lb,ub,'x0',Z0,'options',opts);
% 
% %     Solve NLP
% tic
% [Z,fval,exitflag,info] = solve(Opt);
% timerVal = toc;

%% YALMIP - SNOPT
% Nonlincon = @(Z) nonlcon_snopt(Z,MQ,MR,MR_jerk,N,obstacles,lane_st,ylimit,xk,uk,dt,obstacles_u,r_obs,xr,r_safety_margin);
% % userfun = ;
% Flow = [-inf; ones((length(obstacles))*N,1)*-inf; zeros(3*N,1) ]; %
% Fupp = [ inf; ones((length(obstacles))*N,1)*0; zeros(3*N,1) ];
% xlow = lb;
% xupp = ub;
% % obj_fun; Nonlincon}
% tic 
% [Z,~,info,~,~,~,~,~] = snopt(Z0', xlow', xupp', [], [], Flow, Fupp, [], [],Nonlincon, []);
% fval = objective_func(Z,MQ,MR,MR_jerk,N,obstacles,lane_st,ylimit);
% timerVal = toc;
% exitflag = 1;



end