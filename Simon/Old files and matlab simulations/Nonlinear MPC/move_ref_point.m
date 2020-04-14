function [xref,yref] =  move_ref_point(x, y, xref_final, yref_final, lane, center, max_pred_dist, road_radius, lanewidth)
    x_displaced = x-center(1);
    y_displaced = y-center(2);
    curr_th = acos(x_displaced/(norm([x_displaced; y_displaced])));
    radius_ref = road_radius_frm_lane(lane,road_radius,lanewidth);
    x_ego_ref_lane = cos(curr_th)*radius_ref;
    y_ego_ref_lane=sin(curr_th)*radius_ref;
    th_new = curr_th-max_pred_dist/radius_ref;
    x_ego_ref_lane = radius_ref*cos(th_new);
    y_ego_ref_lane = radius_ref*sin(th_new);
    xref = x_ego_ref_lane+center(1);
    yref = y_ego_ref_lane+center(2);
    if xref_final<xref
       xref = xref_final;
       yref = yref_final;
    end
end
    


