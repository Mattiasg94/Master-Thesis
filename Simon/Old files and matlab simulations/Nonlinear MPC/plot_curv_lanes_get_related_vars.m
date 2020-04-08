function [plot_x_curv,plot_y_curv] = plot_curv_lanes_get_related_vars(road_radius,num_lines,th_max,th_min,lane_offset_y,Length,lanewidth)
plot_x_curv = {};
plot_y_curv = {};
th_lst = [];
r_p = road_radius;
l = 1;

for i = 1:num_lines
    X_p = [];
    Y_p = [];
    d_th = asin(l/(2*r_p));
    th_p = th_min;
    while th_min <= th_p && th_p <= th_max
        x_p = r_p*cos(th_p)+Length/2;
        y_p = r_p*sin(th_p)-(road_radius-lane_offset_y);
        if 0 <= x_p && x_p <= Length
            th_lst = [th_lst th_p];
            X_p = [X_p x_p];
            Y_p = [Y_p y_p];
        end
        th_p = th_p + d_th;
    end
    plot_x_curv{i} = X_p;
    plot_y_curv{i} = Y_p;
    r_p =r_p + lanewidth;
    hold on
    plot(plot_x_curv{i},plot_y_curv{i})
end
xlim([0 40])
ylim([0 4])