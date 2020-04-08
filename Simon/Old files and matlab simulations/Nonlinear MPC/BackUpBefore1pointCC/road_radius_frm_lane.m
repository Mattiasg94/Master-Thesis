function radius = road_radius_frm_lane(lane,road_radius,lanewidth)
    radius=(road_radius-lanewidth/2)+lane*lanewidth;
end