function [x, y]=obs_move_line(dt,lane, v, x, y, road_radius, lanewidth, lane_offset_x, lane_offset_y, lane_border_min)
    radius=road_radius_frm_lane(lane,road_radius,lanewidth);
    x_displaced = x-lane_offset_x;
    y_displaced = y+(lane_border_min-lane_offset_y);
    curr_th = acos(x_displaced/(norm([x_displaced, y_displaced])));
    d = dt*-v;   % v must be flipped! Due to the unit circle.
    th_new = curr_th+d/radius;
    x_displaced = radius*cos(th_new);
    y_displaced = radius*sin(th_new);
    x = x_displaced+lane_offset_x;
    y = y_displaced-(lane_border_min-lane_offset_y);
end