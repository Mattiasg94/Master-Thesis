function [t_impact, arc] = get_intersection_time(x,y,v,x_obs,y_obs,v_obs,r_ego,r_obs,center)
v_obs = -v_obs;

delta_r = abs(r_ego-r_obs);
if r_ego > r_obs
    r_arc = r_obs+delta_r;
else
    r_arc = r_ego+delta_r;
end
x = x-center(1);
y = y-center(2);
x_obs = x_obs-center(1);
y_obs = y_obs-center(2);
th_ego=atan2(y,x);
th_obs=atan2(y_obs,x_obs);
th=abs(th_obs-th_ego);
arc = r_arc*th;
t_impact=arc/(v+v_obs);
end