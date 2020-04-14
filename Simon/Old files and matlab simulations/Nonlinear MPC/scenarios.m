%% Scenario 1 - 3 static obstacels
clc; clear all; close all;

ur = [0; 0];                    % Reference velocity
xr = [40; 0.5; -pi/8];     % Reference state
x0 = [0; 2.25; 0];              % Initial state
u0 = [1;0];                     % Initial input

get_curvs;

obstacles_lanes =  {[1],[2],[1]};      % Lane numbers of obstacles
obstacles = {[20; get_y_from_lane(obstacles_lanes{1}, 20, plot_x_curv,plot_y_curv,lanewidth); 0],...
             [20; get_y_from_lane(obstacles_lanes{2}, 20, plot_x_curv,plot_y_curv,lanewidth); 0],...
             [20; get_y_from_lane(obstacles_lanes{2}, 20, plot_x_curv,plot_y_curv,lanewidth); 0]}; % (x, y ,theta) for obstacle

v_obs = [0; 0; 0];

r_ego = 0.25;
r_obs = 0.25 + r_ego;                % Radius of obstacles
r_safety_margin = 0.2;      % Safety radius, used for soft constraints on obstacle

initFile;
model_DD;

%% Scenario 2 


%% Scenario 3


%% Scenario 4 





