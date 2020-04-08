%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file initializes hyperparameters.                                  %
% The end goal is to be able to control the model from this script alone. %
% Things to add/do here: warm-start toogle, remove close_obstacle         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% !!!! Run from model_DD.m !!!! 

%% Hyperparameters 
N = 10;         % Prediction Horizon
Nsim = 200;     % Simulation steps           
Length = 40;    % Length of track in x direction. (From 0-40)
dt = 0.5;       % Sample rate
ur = [0; 0];    % Reference velocity
xr = [Length/2; 3.25; -pi/8];       % Reference state
x0 = [0; 2.25; 0];             % Initial state
u0 = [0.5;0];                   % Initial input
num_ref_lane = 1;               % Reference lane number
lines_st.lanes = [1; 2; 3];     % Line numbers
road_radius = 1000;             % Radius of road
num_lines = length(lines_st.lanes );        % Number of lanes
lanewidth = 1;                              % Width of lane
lane_border_max = road_radius+num_lines*lanewidth;  % Radius to max border of track
lane_border_min = road_radius;                      % Radius to min border of track
lane_offset_x = Length/2;                           % Offset for x to the middle point of track (x in (0:40)) 
lane_offset_y = 2;                                  % Offset for y to the middle point of track (y in (0:4)) 
lb_x = [0 0 -2*pi];             % Lowerbound states, lb_x = [x y theta]
ub_x = [40 4 2*pi];             % Upperbound states, ub_x = [x y theta]
lb_u = [0 -3];                  % Lowerbound input, lb_u = [v w]
ub_u = [2 3];                   % Upperbound input, lb_u = [v w]
lb = [-inf -inf -inf lb_x -inf -inf lb_u];      % Lowerbound  vec, [x_tilde y_tilde theta_tilde lb_x v_tilde w_tilde lb_u]
ub = [ inf  inf  inf ub_x  inf  inf ub_u];      % Upperbound  vec, [x_tilde y_tilde theta_tilde ub_x v_tilde w_tilde ub_u]
dv = 0.5;              % Max acceleration during one timestep
dw = 1;              % Max angular acceleration during one timestep
Q=[500 0 0;          % x state weight
    0 500 0;         % y state weight
    0 0 1];          % theta state weight
R=[1 0;              % v input weight
    0 1];            % w input weight
R_jerk = 1;          % Term that penalizes the high amounts of jerk [accelerations]
dist_cont = 10;       % Distance for when collision cone shall be activated 
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
lines_st.reference_lane = road_radius_frm_lane(num_ref_lane,road_radius,lanewidth); % Get the radius to the reference lane

%% OBSTACLES
obstacles_lanes =  {[1]}; %,[1]};        % Lane numbers of obstacles
obstacles = {[20; get_y_from_lane(obstacles_lanes{1}, 20, plot_x_curv,plot_y_curv,lanewidth); 0]}; %, [20; get_y_from_lane(obstacles_lanes{2}, 20, plot_x_curv,plot_y_curv,lanewidth); 0]}; % (x, y ,theta) for obstacle
v_obs = [-0.5];
obstacle_radius = [road_radius_frm_lane(obstacles_lanes{1},road_radius,lanewidth)  ]; % road_radius_frm_lane(obstacles_lanes{2},road_radius,lanewidth)];
obstacles_u = {[v_obs(1); -v_obs(1)/obstacle_radius(1)]}; %, [v_obs(2); -v_obs(2)/obstacle_radius(2)]};       % Input values for obstacles
r_ego = 0.25;
r_obs = 0.25 + r_ego;                % Radius of obstacles
r_safety_margin = 0.2;      % Safety radius, used for soft constraints on obstacle

%% Traj. parameters, currently not in use
follow_traj = false;
Y_REF = [11, 10.99, 10.96, 10.91, 10.84, 10.76, 10.65, 10.53, 10.39, 10.24, 10.08, 9.91, 9.72, 9.53, 9.34, 9.14, 8.94, 8.74, 8.55, 8.35, 8.17, 7.99, 7.82, 7.67, 7.53, 7.4, 7.29, 7.19, 7.12, 7.06, 7.02, 7.0, 7.0, 7.03, 7.07, 7.13, 7.21, 7.3, 7.42, 7.55, 7.69, 7.85, 8.02, 8.2, 8.39, 8.58, 8.78, 8.98, 9.17, 9.37, 9.57, 9.76, 9.94, 10.11, 10.27, 10.42, 10.55, 10.67, 10.77, 10.85, 10.92, 10.97, 10.99];
X_REF = [0.0, 0.16, 0.32, 0.48, 0.65, 0.81, 0.97, 1.13, 1.29, 1.45, 1.61, 1.77, 1.94, 2.1, 2.26, 2.42, 2.58, 2.74, 2.9, 3.06, 3.23, 3.39, 3.55, 3.71, 3.87, 4.03, 4.19, 4.35, 4.52, 4.68, 4.84, 5.0, 5.16, 5.32, 5.48, 5.65, 5.81, 5.97, 6.13, 6.29, 6.45, 6.61, 6.77, 6.94, 7.1, 7.26, 7.42, 7.58, 7.74, 7.9, 8.06, 8.23, 8.39, 8.55, 8.71, 8.87, 9.03, 9.19, 9.35, 9.52, 9.68, 9.84, 10.0];
THETA_REF=[0.0, -0.07841, -0.156035, -0.2321, -0.3058475, -0.37654, -0.44347, -0.5059675, -0.56341, -0.6152225, -0.66089, -0.6999525, -0.7320225, -0.7567775, -0.77397, -0.78343, -0.7850625, -0.7788525, -0.7648575, -0.7432225, -0.71416, -0.6779625, -0.6349925, -0.585675, -0.5305075, -0.47004, -0.404875, -0.3356625, -0.2631, -0.187905, -0.110835, -0.0326575, 0.0458475, 0.1238925, 0.2007025, 0.275505, 0.347555, 0.4161325, 0.4805525, 0.54017, 0.5943925, 0.6426725, 0.684535, 0.719555, 0.7473875, 0.76775, 0.7804425, 0.7853375, 0.782385, 0.7716175, 0.7531375, 0.7271325, 0.6938625, 0.6536625, 0.6069275, 0.55413, 0.495795, 0.4325075, 0.3648975, 0.2936425, 0.2194525, 0.14307, 0.0652575];
if follow_traj
    x0=[X_REF(1);Y_REF(1);THETA_REF(1)];
end

%% Plot obstacles and reference
plot_obstacles = plot(1); textbox=plot(1); plot_obstacles_radius=plot(1); plot_obstacles_safety_radius = plot(1);
if follow_traj
    viscircles([X_REF(end),Y_REF(end)],0.1,'Color','y','Linewidth',5);
else
    viscircles([xr(1),xr(2)],0.1,'Color','y','Linewidth',5);
end

%% Initializiation variables
Vfval = zeros(Nsim,1);              % Used for infeasibility check
Vfval(1) = inf;                     % Used for infeasibility check
time_it = 0;                        % Iterable used to find mean optimization time
feasible = 1;                       % Variable to check if solution is feasible.

%% Builds all necessary weight matrices vectors:
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta,MR_jerk] = setup(x0,u0,xr,ur,ub,lb,Nsim,N,R_jerk,Q,R);