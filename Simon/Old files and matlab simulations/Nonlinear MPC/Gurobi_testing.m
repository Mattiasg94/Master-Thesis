%%
% clc; clear;
% 
% model.A = sparse([1 1 0; 0 1 1]);
% model.obj = [1 2 3];
% model.modelsense = 'Max';
% model.rhs = [1 1];
% 
% result = gurobi(model);
% 
% disp(result.objval);
% disp(result.x);
% 
% % Alterantive representation of A - as sparse triplet matrix
% i = [1; 1; 2; 2];
% j = [1; 2; 2; 3];
% x = [1; 1; 1; 1];
% model.A = sparse(i, j, x, 2, 3);
% 
% 
% % Set some parameters
% params.method = 0; % 1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier, 3=concurrent, 4=deterministic concurrent, 5=deterministic concurrent simplex 
% % params.timelimit = 100;
% % params = [];
% 
% result = gurobi(model, params);
% 
% disp(result.objval);
% disp(result.x)
% 



%%
clc; clear;
% Four nonneg. variables x, y, u, one linear constraint u + 4*v <= 9
m.varnames = {'x', 'y', 'u'};
m.lb = [-10*ones(2, 1); -1];
m.ub = [10*ones(2, 1); 1];
m.A = sparse([1, 0, 0]);
m.rhs = 8;

% Objective
m.modelsense = 'min';
m.obj = [0; 1; 3];

% Approximate u \approx exp(x), equispaced points in [0, xmax], xmax = log(9)
m.genconpwl(1).xvar = 1;
m.genconpwl(1).yvar = 3;
m.genconpwl(1).xpts = -10:1e-3:10;
m.genconpwl(1).ypts = sin(m.genconpwl(1).xpts);

% Solve and print solution
result = gurobi(m);
printsol(result.objval, result.x(1), result.x(2), result.x(3));

function printsol(objval, x, y, u)
    fprintf('x = %g, u = %g\n', x, u);
    fprintf('y = %g, v = %g\n', y);
    fprintf('Obj = %g\n', objval);

    % Calculate violation of exp(x) + 4 sqrt(y) <= 9
    vio = 0;
    if vio < 0
        vio = 0;
    end
    fprintf('Vio = %g\n', vio);
end




