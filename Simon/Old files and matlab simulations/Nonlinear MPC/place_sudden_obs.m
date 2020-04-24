function [x_obs,y_obs_new,x,y] = place_sudden_obs(x,y,x_obs,y_obs,lane,plot_x_curv,plot_y_curv,lanewidth)
y_obs_new=get_y_from_lane(lane, x,plot_x_curv,plot_y_curv,lanewidth);
    dist_to_obs=sqrt((x-(-x_obs))^2+(y-y_obs_new)^2);
    if dist_to_obs<5.5
        x_obs = -x_obs;
        y_obs_new = y_obs_new;
        x = x;
        y = y;
    else
        x_obs = x_obs;
        y_obs = y_obs;
        x=-10;
        y=-10;
    end
    
end