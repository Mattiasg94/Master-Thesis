function deactivate_cone = cone_logic(obs_dist, ego_dist, u, v_obs, x, y, x_obs, y_obs, dist_cont)
%% Conditions:
passed_obs = (obs_dist>ego_dist);
obs_faster_than_ego = norm(u) < norm(v_obs);

if passed_obs
    obs_driving_towards = false;
else
    if v_obs <= 0
        obs_driving_towards = true;
    else
        obs_driving_towards = false;
    end
end

if obs_driving_towards
    skip_due_to_dir_and_vel = false;
else
    if obs_faster_than_ego
        skip_due_to_dir_and_vel = true;
    else
        skip_due_to_dir_and_vel = false;
    end
end

skip_due_far_away = (sqrt((x-x_obs)^2+(y-y_obs)^2) > dist_cont);

skip = skip_due_to_dir_and_vel || skip_due_far_away;

if passed_obs
    deactivate_cone = true;
else
    deactivate_cone = skip;
end
