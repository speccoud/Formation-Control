function f3 = control_parameter_f3(a_f, V_avoid_obstacle, V_follow_wall)
% Implement wall-following conditions here
wall_following_on = true; % Change this based on the conditions for wall-following behavior

if wall_following_on
    f3 = a_f;
else
    f3 = 0;
end
end
