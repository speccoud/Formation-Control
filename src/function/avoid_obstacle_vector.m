function V = avoid_obstacle_vector(x_i, y_i, x_o, y_o, V_move_to_goal)
robot_to_obstacle = [x_o - x_i; y_o - y_i];
sign_pm = sign(dot(V_move_to_goal, robot_to_obstacle, 1));
V = (1 / norm(robot_to_obstacle)) * [sign_pm * (y_o - y_i); -sign_pm * (x_o - x_i)];
end
