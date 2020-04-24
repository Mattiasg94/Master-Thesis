%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file initializes hyperparameters.                                  %
% The end goal is to be able to control the model from this script alone. %
% Things to add/do here: warm-start toogle, remove close_obstacle         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% !!!! Run from here !!!!
clc;
clear all;
close all;

%% Scenarios
test_scenario = false;
scenario1 = true;
scenario2 = false;
scenario3 = false; % No logic!
scenario4 = false;

%% OBSTACLES
if test_scenario
    
    
elseif scenario1
    
    %% Hyperparameters
    options = optimoptions('fmincon','Algorithm','sqp','Display','off','MaxFunctionEvaluations',4000,...
        'MaxIterations',4000,'FiniteDifferenceType','central','FunctionTolerance', 1.0000e-8,...
        'OptimalityTolerance',1.0000e-4,'ConstraintTolerance', 1.0000e-02,'UseParallel', true,'ScaleProblem',true,...
        'HessianApproximation','lbfgs','StepTolerance',1.0000e-06 ); % ,'MaxSQPIter',2000
    
    
    % Interior-point kan fungerar med extra tuning. Bör skapa separata
    % scenario filer för alla.
    % HessianApproximation Ser ingen större skillnad
    % MaxSQPIter ingen skillnad
    
    N = 10;          % Prediction Horizon
    Nsim = 100;      % Simulation steps
    Length = 40;    % Length of track in x direction. (From 0-40)
    dt = 0.5;       % Sample rate
    ur = [0; 0];    % Reference velocity
    
    x0 = [0; 514.4789e-003; pi/20];              % Initial state
    % [0; 2.3079; pi/20];
    % x0 = [0; 3.3039; pi/20];
    
    u0 = [0.1;0];                   % Initial input
    lines_st.lanes = [1; 2; 3];     % Line numbers
    road_radius = 100;             % Radius of road
    num_lines = length(lines_st.lanes);        % Number of lanes
    lanewidth = 1;                              % Width of lane
    lane_border_max = road_radius+num_lines*lanewidth;  % Radius to max border of track
    lane_border_min = road_radius;                      % Radius to min border of track
    lane_offset_x = Length/2;                           % Offset for x to the middle point of track (x in (0:40))
    lane_offset_y = 2;                                  % Offset for y to the middle point of track (y in (0:4))
    lb_x = [-inf 0 -2*pi];             % Lowerbound states, lb_x = [x y theta]
    ub_x = [inf 5 2*pi];             % Upperbound states, ub_x = [x y theta]
    lb_u = [0.1 -3];                  % Lowerbound input, lb_u = [v w]
    ub_u = [3 3];                   % Upperbound input, lb_u = [v w]
    lb = [-inf -inf -inf lb_x -inf -inf lb_u];      % Lowerbound  vec, [x_tilde y_tilde theta_tilde lb_x v_tilde w_tilde lb_u]
    ub = [ inf  inf  inf ub_x  inf  inf ub_u];      % Upperbound  vec, [x_tilde y_tilde theta_tilde ub_x v_tilde w_tilde ub_u]
    dv = 0.5;              % Max acceleration during one timestep
    dw = 0.5;
%     Q=[45 0 0;          % x state weight
%         0 15 0;         % y state weight
%         0 0 0];          % theta state weight
        Q=[10 0 0;          % x state weight
        0 10 0;         % y state weight
        0 0 0];          % theta state weight
    Qt=50*Q;
    R=[0 0;              % v input weight
        0 0];            % w input weight
    R_jerk = 1;          % Term that penalizes the high amounts of jerk [accelerations]
    dist_cont = 5;       % Distance for when collision cone shall be activated
    %     barrier_weight = 150; % LINEAR BARRIER!
    barrier_weight = 35; % Noninear BARRIER!
    grade = 4;           % Grade of curve fitting polynomial, used for infeasibility check
    Error_is_small = true;  % Boolean to control if curvefitting error is small enough
    u_saved = u0;           % Initial u_saved vector, used to store values from previously feasible solutions
    iterSaved_u = 1;        % Iterable to be able to iterate over the stored u_saved values
    warmstart = true;       % Initial warmstart toggle
    center=[lane_offset_x; lane_offset_y-lane_border_min];
    
    %% Plot Curv-Lanes
    th_max = pi*3/4;        % Describes the curvature of the trajectory as the interval between th_max and th_min
    th_min = pi/4;          % Describes the curvature of the trajectory as the interval between th_max and th_min
    [plot_x_curv,plot_y_curv] = plot_curv_lanes_get_related_vars(road_radius,...
        num_lines,th_max,th_min,lane_offset_y,Length,lanewidth);         % Extract the lane contours and its values
    lines_st.reference_lane = road_radius_frm_lane(3,road_radius,lanewidth); % Get the radius to the reference lane
    
    obstacles_lanes =  {1,2,1}; %,[1]};        % Lane numbers of obstacles
    
    % NOTERA ATT X MÅSTE ÄNDRAS I GET_Y_LANE OCKSÅ
    obstacles = {[13; get_y_from_lane(obstacles_lanes{1}, 13, plot_x_curv,plot_y_curv,lanewidth); 0],...
        [20; get_y_from_lane(obstacles_lanes{2}, 20, plot_x_curv,plot_y_curv,lanewidth); 0],...
        [27; get_y_from_lane(obstacles_lanes{3}, 27, plot_x_curv,plot_y_curv,lanewidth); 0]};
    
    v_obs = [0; 0; 0];
    for i = 1:length(obstacles)
        obstacle_radius(i) = [road_radius_frm_lane(obstacles_lanes{i},road_radius,lanewidth)  ];
        obstacles_u{i} = [v_obs(i); -v_obs(i)/obstacle_radius(i)];
    end
    
    xr = [Length; get_y_from_lane(1, Length, plot_x_curv,plot_y_curv,lanewidth) ; -pi/8];
    reference_lane_number = 1;
    xref_final = xr(1);
    yref_final = xr(2);
    th_final = xr(3);
    
elseif scenario2
    options = optimoptions('fmincon','Algorithm','sqp','Display','off','MaxFunctionEvaluations',4000,...
        'MaxIterations',4000,'FiniteDifferenceType','central','FunctionTolerance', 1.0000e-8,...
        'OptimalityTolerance',1.0000e-04,'ConstraintTolerance', 1.0000e-04,'UseParallel',...
        true,'ScaleProblem',true,'HessianApproximation','lbfgs'); %,'TolCon',1e-6
    
    %% Hyperparameters
    N = 10;          % Prediction Horizon
    Nsim = 250;      % Simulation steps
    Length = 40;    % Length of track in x direction. (From 0-40)
    dt = 0.5;       % Sample rate
    ur = [0; 0];    % Reference velocity
    
    x0 = [0; 514.4789e-003; pi/20];              % Initial state
    % [0; 2.3079; pi/20];
    % x0 = [0; 3.3039; pi/20];
    
    u0 = [0.1;0];                   % Initial input
    lines_st.lanes = [1; 2; 3];     % Line numbers
    road_radius = 100;             % Radius of road
    num_lines = length(lines_st.lanes);        % Number of lanes
    lanewidth = 1;                              % Width of lane
    lane_border_max = road_radius+num_lines*lanewidth;  % Radius to max border of track
    lane_border_min = road_radius;                      % Radius to min border of track
    lane_offset_x = Length/2;                           % Offset for x to the middle point of track (x in (0:40))
    lane_offset_y = 2;                                  % Offset for y to the middle point of track (y in (0:4))
    lb_x = [-inf 0 -2*pi];             % Lowerbound states, lb_x = [x y theta]
    ub_x = [inf 5 2*pi];             % Upperbound states, ub_x = [x y theta]
    lb_u = [0.1 -3];                  % Lowerbound input, lb_u = [v w]
    ub_u = [3 3];                   % Upperbound input, lb_u = [v w]
    lb = [-inf -inf -inf lb_x -inf -inf lb_u];      % Lowerbound  vec, [x_tilde y_tilde theta_tilde lb_x v_tilde w_tilde lb_u]
    ub = [ inf  inf  inf ub_x  inf  inf ub_u];      % Upperbound  vec, [x_tilde y_tilde theta_tilde ub_x v_tilde w_tilde ub_u]
    dv = 0.5;              % Max acceleration during one timestep
    dw = 0.5;
    Q=[45 0 0;          % x state weight
        0 10 0;         % y state weight
        0 0 1];          % theta state weight
    Qt=50*Q;
    R=[1 0;              % v input weight
        0 1];            % w input weight
    R_jerk = 1;          % Term that penalizes the high amounts of jerk [accelerations]
    dist_cont = 5;       % Distance for when collision cone shall be activated
    %     barrier_weight = 150;
    barrier_weight = 35;
    grade = 4;           % Grade of curve fitting polynomial, used for infeasibility check
    Error_is_small = true;  % Boolean to control if curvefitting error is small enough
    u_saved = u0;           % Initial u_saved vector, used to store values from previously feasible solutions
    iterSaved_u = 1;        % Iterable to be able to iterate over the stored u_saved values
    warmstart = true;       % Initial warmstart toggle
    center=[lane_offset_x; lane_offset_y-lane_border_min];
    
    %% Plot Curv-Lanes
    th_max = pi*3/4;        % Describes the curvature of the trajectory as the interval between th_max and th_min
    th_min = pi/4;          % Describes the curvature of the trajectory as the interval between th_max and th_min
    [plot_x_curv,plot_y_curv] = plot_curv_lanes_get_related_vars(road_radius,...
        num_lines,th_max,th_min,lane_offset_y,Length,lanewidth);         % Extract the lane contours and its values
    lines_st.reference_lane = road_radius_frm_lane(3,road_radius,lanewidth); % Get the radius to the reference lane
    
    obstacles_lanes =  {1,2}; %,[1]};        % Lane numbers of obstacles
    obstacles = {[13; get_y_from_lane(obstacles_lanes{1}, 13, plot_x_curv,plot_y_curv,lanewidth); 0],...
        [30; get_y_from_lane(obstacles_lanes{2}, 30, plot_x_curv,plot_y_curv,lanewidth); 0]}; %, [20; get_y_from_lane(obstacles_lanes{2}, 20, plot_x_curv,plot_y_curv,lanewidth); 0]}; % (x, y ,theta) for obstacle
    v_obs = [0.7; -0.5];
    for i = 1:length(obstacles)
        obstacle_radius(i) = [road_radius_frm_lane(obstacles_lanes{i},road_radius,lanewidth)  ]; % road_radius_frm_lane(obstacles_lanes{2},road_radius,lanewidth)];
        obstacles_u{i} = [v_obs(i); -v_obs(i)/obstacle_radius(i)]; %, [v_obs(2); -v_obs(2)/obstacle_radius(2)]};       % Input values for obstacles
    end
    
    xr = [Length; get_y_from_lane(1, Length, plot_x_curv,plot_y_curv,lanewidth) ; -pi/8];       % Reference state
    reference_lane_number = 1;
    xref_final = xr(1);
    yref_final = xr(2);
    th_final = xr(3);
    
elseif scenario3
%     options = optimoptions('fmincon','Algorithm','sqp','Display','off','MaxFunctionEvaluations',40000,...
%         'MaxIterations',40000,'FiniteDifferenceType','central','FunctionTolerance', 1.0000e-8,...
%         'OptimalityTolerance',1.0000e-04,'ConstraintTolerance', 1.0000e-02,'UseParallel', true,'ScaleProblem',true); %,'TolCon',1e-6
%     
%     %% Hyperparameters
%     N = 10;          % Prediction Horizon
%     Nsim = 80;      % Simulation steps
%     Length = 40;    % Length of track in x direction. (From 0-40)
%     dt = 0.5;       % Sample rate
%     ur = [0; 0];    % Reference velocity
%     
%     x0 = [0; 2.4079; pi/20];              % Initial state
%     % [0; 2.3079; pi/20];
%     % x0 = [0; 3.3039; pi/20];
%     
%     u0 = [0;0];                   % Initial input
%     lines_st.lanes = [1; 2; 3];     % Line numbers
%     road_radius = 1000;             % Radius of road
%     num_lines = length(lines_st.lanes);        % Number of lanes
%     lanewidth = 1;                              % Width of lane
%     lane_border_max = road_radius+num_lines*lanewidth;  % Radius to max border of track
%     lane_border_min = road_radius;                      % Radius to min border of track
%     lane_offset_x = Length/2;                           % Offset for x to the middle point of track (x in (0:40))
%     lane_offset_y = 2;                                  % Offset for y to the middle point of track (y in (0:4))
%     lb_x = [-inf 0 -2*pi];             % Lowerbound states, lb_x = [x y theta]
%     ub_x = [inf 5 2*pi];             % Upperbound states, ub_x = [x y theta]
%     lb_u = [0 -3];                  % Lowerbound input, lb_u = [v w]
%     ub_u = [3 3];                   % Upperbound input, lb_u = [v w]
%     lb = [-inf -inf -inf lb_x -inf -inf lb_u];      % Lowerbound  vec, [x_tilde y_tilde theta_tilde lb_x v_tilde w_tilde lb_u]
%     ub = [ inf  inf  inf ub_x  inf  inf ub_u];      % Upperbound  vec, [x_tilde y_tilde theta_tilde ub_x v_tilde w_tilde ub_u]
%     dv = 1;              % Max acceleration during one timestep
%     dw = 1;              % Max angular acceleration during one timestep
%     Q=[45 0 0;          % x state weight
%         0 15 0;         % y state weight
%         0 0 1];          % theta state weight
%     Qt=50*Q;
%     R=[1 0;              % v input weight
%         0 1];            % w input weight
%     R_jerk = 1;          % Term that penalizes the high amounts of jerk [accelerations]
%     dist_cont = 5;       % Distance for when collision cone shall be activated
%     barrier_weight = 10;
%     grade = 4;           % Grade of curve fitting polynomial, used for infeasibility check
%     Error_is_small = true;  % Boolean to control if curvefitting error is small enough
%     u_saved = u0;           % Initial u_saved vector, used to store values from previously feasible solutions
%     iterSaved_u = 1;        % Iterable to be able to iterate over the stored u_saved values
%     warmstart = true;       % Initial warmstart toggle
%     center=[lane_offset_x; lane_offset_y-lane_border_min];
%     
%     %% Plot Curv-Lanes
%     th_max = pi*3/4;        % Describes the curvature of the trajectory as the interval between th_max and th_min
%     th_min = pi/4;          % Describes the curvature of the trajectory as the interval between th_max and th_min
%     [plot_x_curv,plot_y_curv] = plot_curv_lanes_get_related_vars(road_radius,...
%         num_lines,th_max,th_min,lane_offset_y,Length,lanewidth);         % Extract the lane contours and its values
%     lines_st.reference_lane = road_radius_frm_lane(3,road_radius,lanewidth); % Get the radius to the reference lane
%     
%     obstacles_lanes =  {1,2}; %,[1]};        % Lane numbers of obstacles
%     obstacles = {[20; get_y_from_lane(obstacles_lanes{1}, 20, plot_x_curv,plot_y_curv,lanewidth); 0],...
%         [20; get_y_from_lane(obstacles_lanes{2}, 20, plot_x_curv,plot_y_curv,lanewidth); 0]}; %, [20; get_y_from_lane(obstacles_lanes{2}, 20, plot_x_curv,plot_y_curv,lanewidth); 0]}; % (x, y ,theta) for obstacle
%     v_obs = [0; 0];
%     for i = 1:length(obstacles)
%         obstacle_radius(i) = [road_radius_frm_lane(obstacles_lanes{i},road_radius,lanewidth)  ]; % road_radius_frm_lane(obstacles_lanes{2},road_radius,lanewidth)];
%         obstacles_u{i} = [v_obs(i); -v_obs(i)/obstacle_radius(i)]; %, [v_obs(2); -v_obs(2)/obstacle_radius(2)]};       % Input values for obstacles
%     end
%     
%     xr = [Length; get_y_from_lane(1, Length, plot_x_curv,plot_y_curv,lanewidth) ; -pi/8];       % Reference state
%     reference_lane_number = 1;
%     xref_final = xr(1);
%     yref_final = xr(2);
%     th_final = xr(3);
    
elseif scenario4
    options = optimoptions('fmincon','Algorithm','sqp','Display','off','MaxFunctionEvaluations',4000,...
        'MaxIterations',4000,'FiniteDifferenceType','central','FunctionTolerance', 1.0000e-8,...
        'OptimalityTolerance',1.0000e-04,'ConstraintTolerance', 1.0000e-02,'UseParallel', true,...
        'ScaleProblem',true,'HessianApproximation','lbfgs'); %,'TolCon',1e-6
    
    %% Hyperparameters
    N = 10;          % Prediction Horizon
    Nsim = 150;      % Simulation steps
    Length = 40;    % Length of track in x direction. (From 0-40)
    dt = 0.5;       % Sample rate
    ur = [0; 0];    % Reference velocity
    
    x0 = [0; 514.4789e-003; pi/20];              % Initial state
    % [0; 2.3079; pi/20];
    % x0 = [0; 3.3039; pi/20];
    
    u0 = [0.1;0];                   % Initial input
    lines_st.lanes = [1; 2; 3];     % Line numbers
    road_radius = 100;             % Radius of road
    num_lines = length(lines_st.lanes);        % Number of lanes
    lanewidth = 1;                              % Width of lane
    lane_border_max = road_radius+num_lines*lanewidth;  % Radius to max border of track
    lane_border_min = road_radius;                      % Radius to min border of track
    lane_offset_x = Length/2;                           % Offset for x to the middle point of track (x in (0:40))
    lane_offset_y = 2;                                  % Offset for y to the middle point of track (y in (0:4))
    lb_x = [-inf 0 -2*pi];             % Lowerbound states, lb_x = [x y theta]
    ub_x = [inf 5 2*pi];             % Upperbound states, ub_x = [x y theta]
    lb_u = [0.1 -3];                 % Lowerbound input, lb_u = [v w]
    ub_u = [3 3];                    % Upperbound input, lb_u = [v w]
    lb = [-inf -inf -inf lb_x -inf -inf lb_u];      % Lowerbound  vec, [x_tilde y_tilde theta_tilde lb_x v_tilde w_tilde lb_u]
    ub = [ inf  inf  inf ub_x  inf  inf ub_u];      % Upperbound  vec, [x_tilde y_tilde theta_tilde ub_x v_tilde w_tilde ub_u]
    dv = 0.5;              % Max acceleration during one timestep
    dw = 0.5;              % Max angular acceleration during one timestep
    Q=[50 0 0;          % x state weight
        0 15 0;         % y state weight
        0 0 0];          % theta state weight
    Qt=50*Q;
    R=[1 0;              % v input weight
        0 1];            % w input weight
    R_jerk = 1;          % Term that penalizes the high amounts of jerk [accelerations]
    dist_cont = 5;       % Distance for when collision cone shall be activated
    %     barrier_weight = 10;
    barrier_weight = 35;
    grade = 4;           % Grade of curve fitting polynomial, used for infeasibility check
    Error_is_small = true;  % Boolean to control if curvefitting error is small enough
    u_saved = u0;           % Initial u_saved vector, used to store values from previously feasible solutions
    iterSaved_u = 1;        % Iterable to be able to iterate over the stored u_saved values
    warmstart = true;       % Initial warmstart toggle
    center=[lane_offset_x; lane_offset_y-lane_border_min];
    
    %% Plot Curv-Lanes
    th_max = pi*3/4;        % Describes the curvature of the trajectory as the interval between th_max and th_min
    th_min = pi/4;          % Describes the curvature of the trajectory as the interval between th_max and th_min
    [plot_x_curv,plot_y_curv] = plot_curv_lanes_get_related_vars(road_radius,...
        num_lines,th_max,th_min,lane_offset_y,Length,lanewidth);         % Extract the lane contours and its values
    lines_st.reference_lane = road_radius_frm_lane(3,road_radius,lanewidth); % Get the radius to the reference lane
    
    %% Obst:
    obstacles_lanes =  {1}; %,[1]};        % Lane numbers of obstacles
    obstacles = {[-16; get_y_from_lane(obstacles_lanes{1}, -16, plot_x_curv,plot_y_curv,lanewidth); 0]};
%     ,...
%         [-100; get_y_from_lane(obstacles_lanes{2}, -100, plot_x_curv,plot_y_curv,lanewidth); 0]}; %, [20; get_y_from_lane(obstacles_lanes{2}, 20, plot_x_curv,plot_y_curv,lanewidth); 0]}; % (x, y ,theta) for obstacle
    v_obs = [0];
    for i = 1:length(obstacles)
        obstacle_radius(i) = [road_radius_frm_lane(obstacles_lanes{i},road_radius,lanewidth)  ]; % road_radius_frm_lane(obstacles_lanes{2},road_radius,lanewidth)];
        obstacles_u{i} = [v_obs(i); -v_obs(i)/obstacle_radius(i)]; %, [v_obs(2); -v_obs(2)/obstacle_radius(2)]};       % Input values for obstacles
    end
    
    xr = [Length; get_y_from_lane(1, Length, plot_x_curv,plot_y_curv,lanewidth) ; -pi/8];       % Reference state
    reference_lane_number = 1;
    xref_final = xr(1);
    yref_final = xr(2);
    th_final = xr(3);
    
end

r_ego = 0.25;
r_obs = 0.25 + r_ego;               % Radius of obstacles
r_safety_margin = 0.2;              % Safety radius, used for soft constraints on obstacle

%% Traj. parameters, currently not in use
follow_traj = false;


%% Plot obstacles and reference
plot_obstacles = plot(1); textbox=plot(1); plot_obstacles_radius=plot(1); plot_obstacles_safety_radius = plot(1);
if follow_traj
    viscircles([X_REF(end),Y_REF(end)],0.1,'Color','y','Linewidth',5);
    % else
    %     viscircles([xr(1),xr(2)],0.1,'Color','y','Linewidth',5);
end

%% Initializiation variables
Vfval = zeros(Nsim,1);              % Used for infeasibility check
Vfval(1) = inf;                     % Used for infeasibility check
time_it = 0;                        % Iterable used to find mean optimization time
feasible = 1;                       % Variable to check if solution is feasible.

%% Builds all necessary weight matrices vectors:
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta,MR_jerk] = setup(x0,u0,xr,ur,ub,lb,Nsim,N,R_jerk,Q,R);

%% Run MPC
xlim([-0.5 40.5])
ylim([0 5])
impactPlot = [];
ref_plot = [];
ego_point = [];
ego_heading = [];
output_saved = [];
model_DD;