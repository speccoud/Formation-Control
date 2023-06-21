function V = follow_wall_vector(x_i, y_i, x_o, y_o)
sign_pm = 1; % Change this based on the relationship between the robot's moving direction and the obstacles
V = (1 / sqrt((x_o - x_i).^2 + (y_o - y_i).^2)) * [sign_pm * (y_o - y_i); -sign_pm * (x_o - x_i)];
end
