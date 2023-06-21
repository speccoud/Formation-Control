function [x_o, y_o, d_0] = nearest_obstacle(x_i, y_i, obstacles)
% Calculate distances from each obstacle to robot
robot_position = [x_i, y_i];
distances = arrayfun(@(x, y, r) sqrt((x - robot_position(1))^2 + (y - robot_position(2))^2) - r, obstacles(:, 1), obstacles(:, 2), obstacles(:, 3));

% Find the nearest obstacle and its distance
[d_0, idx] = min(distances);
x_o = obstacles(idx, 1);
y_o = obstacles(idx, 2);
end
