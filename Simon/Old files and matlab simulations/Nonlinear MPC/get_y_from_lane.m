function y = get_y_from_lane(lane_idx, x,plot_x_curv,plot_y_curv,lanewidth)
    absx_updt = inf;
    for i = 1:length(plot_x_curv{lane_idx})
        absx = abs(plot_x_curv{lane_idx}(i)-x);
        if absx_updt >=  absx
            absx_updt = absx;
            save_idx = i;
        end
    end
    y = plot_y_curv{lane_idx}(save_idx)+lanewidth/2;
end