th_max = pi*3/4;        % Describes the curvature of the trajectory as the interval between th_max and th_min
th_min = pi/4;          % Describes the curvature of the trajectory as the interval between th_max and th_min
[plot_x_curv,plot_y_curv] = plot_curv_lanes_get_related_vars(road_radius,...
                   num_lines,th_max,th_min,lane_offset_y,Length,lanewidth);         % Extract the lane contours and its values
