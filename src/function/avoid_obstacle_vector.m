function V = avoid_obstacle_vector(x_i, y_i, x_o, y_o, V_move_to_goal)
robot_to_obstacle = [ (y_o - y_i); -1 * (x_o - x_i)];
% sign_pm = sign(dot(V_move_to_goal, robot_to_obstacle, 1));
V = (robot_to_obstacle / norm(robot_to_obstacle));
end
