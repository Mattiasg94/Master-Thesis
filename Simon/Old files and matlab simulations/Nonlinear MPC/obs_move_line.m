function [x, y,theta,w]=obs_move_line(dt, lane, v, x, y, center,road_radius,lanewidth)
    v = -v;
    radius = road_radius_frm_lane(lane,road_radius,lanewidth);
    x_displaced = x-center(1);
    y_displaced = y-center(2);
    curr_th = acos(x_displaced/(norm([x_displaced; y_displaced])));
    d = dt*v;
    th_new = curr_th+d/radius;
    x_displaced = radius*cos(th_new);
    y_displaced = radius*sin(th_new);
    x = x_displaced + center(1);
    y = y_displaced + center(2);
    theta = th_new;
    w = abs(th_new-curr_th)/dt; % TODO which is the right way?
end 




