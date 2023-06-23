function V = move_to_goal_vector(x_i, y_i, x_t, y_t)
V = [x_t - x_i; y_t - y_i] / norm([x_t - x_i; y_t - y_i]);
end
