function [t_impact, arc] = get_intersection_time(x,y,v,x_obs,y_obs,v_obs,r_circ_ego,r_circ_obs,center)
v_obs = -v_obs;

if r_circ_ego > r_circ_obs
    radius_avg = r_circ_obs+(r_circ_ego-r_circ_obs);
else
    radius_avg = r_circ_obs-(r_circ_ego-r_circ_obs);
end
x = x-center(1);
y = y-center(2);
x_obs = x_obs-center(1);
y_obs = y_obs-center(2);
th_ego=atan2(y,x);
th_obs=atan2(y_obs,x_obs);
th=sqrt(th_obs-th_ego)^2;
arc = radius_avg*th;
t_impact=arc/(v+v_obs);
end

%% Python code, for reference
%     v_obs=-v_obs
%     radius_ego=cs.sqrt((x-center[0])**2+(y-center[1])**2)
%     radius_obs = road_radius_frm_lane(lane)
%     radius_avg=cs.if_else(radius_ego>radius_obs,radius_obs+(radius_ego-radius_obs),radius_obs-(radius_ego-radius_obs))
%     (x,y)=(x-center[0],y-center[1])
%     (x_obs,y_obs)=(x_obs-center[0],y_obs-center[1])
%     th_ego=cs.atan2(y,x)
%     th_obs=cs.atan2(y_obs,x_obs)
%     th=cs.sqrt((th_obs-th_ego)**2)
%     arc=radius_avg*th
%     t_impact=arc/(v+v_obs)

