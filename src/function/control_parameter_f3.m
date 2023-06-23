function f3 = control_parameter_f3(a_f, d_0, wall_follow_threshold)
% Implement wall-following conditions here
wall_following_on = d_0 <= wall_follow_threshold; % Change this based on the conditions for wall-following behavior

if wall_following_on
    f3 = a_f;
else
    f3 = 0;
end
end
