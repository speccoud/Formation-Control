%% -- Main ---
close all;
clear all;
clc;

% Parameters
a_m = 1;   % Amplitude of the move-to-goal force; affects the strength of the attraction towards the goal
b_m = 3;   % Distance at which the move-to-goal force starts to diminish; beyond this distance, the force remains constant

a_0 = 1;   % Amplitude of the avoid-obstacle force; affects the strength of the repulsion from the obstacles
b_0 = 7;   % Distance at which the avoid-obstacle force starts to diminish; beyond this distance, the force is zero
b_f = 2;   % Distance at which the avoid-obstacle force reaches its maximum value; below this distance, the force is zero

a_f = 1;   % Amplitude of the follow-wall force; affects the strength of the attraction towards the wall when wall-following behavior is active

movement_speed = 0.05;

% Robot positions
robot_positions = [
    -5, -5;
    -4, -5;
    -3, -5;
    -2, -5;
    -1, -5;
    -5, -3;
    -4, -3;
    -3, -3;
    -2, -3;
    -1, -3;
    -5, -1;
    -4, -1;
    -3, -1;
    -2, -1;
    -1, -1;
    -5,  1;
    -4,  1;
    -3,  1;
    -2,  1;
    -1,  1;
    ];
num_robots = size(robot_positions, 1);
x_i = robot_positions(:, 1);
y_i = robot_positions(:, 2);
agent_colors = lines(num_robots);

% Goal positions
x_t = 12;
y_t = 12;

% Obstacles (x_o, y_o, radius)
obstacles = [
    2, 2, 2;
    5, 5, 1;
    8, 8, 2
    ];

% Simulation steps
steps = 1200;

% Initialize plot
figure;
hold on;
xlim([-5, 15]);
ylim([-5, 15]);
plot_handles = zeros(num_robots, 1);
for i = 1:num_robots
    plot_handles(i) = plot(x_i(i), y_i(i), 'o', 'Color', agent_colors(i, :));
end
plot(x_t, y_t, 'ro');
for obs = obstacles'
    rectangle('Position', [obs(1) - obs(3), obs(2) - obs(3), obs(3) * 2, obs(3) * 2], 'Curvature', 1);
end


%% --- Movement Loop ---
for step = 1:steps
    for i = 1:num_robots
        % Calculate move_to_goal vector for each robot
        V_move_to_goal = move_to_goal_vector(x_i(i), y_i(i), x_t, y_t);

        % Calculate nearest obstacle and distance
        [x_o, y_o, d_0] = nearest_obstacle(x_i(i), y_i(i), obstacles);

        % Calculate avoid_obstacle vector for each robot
        V_avoid_obstacle = avoid_obstacle_vector(x_i(i), y_i(i), x_o, y_o, V_move_to_goal);

        % Calculate follow_wall vector
        V_follow_wall = follow_wall_vector(x_i(i), y_i(i), x_o, y_o);

        % Calculate control parameters
        f1 = control_parameter_f1(d_0, a_m, b_m);
        f2 = control_parameter_f2(d_0, a_0, b_0, b_f);
        f3 = control_parameter_f3(a_f, V_avoid_obstacle, V_follow_wall);

        % Calculate direction vector
        V_direction = f1 * V_move_to_goal + f2 * V_avoid_obstacle + f3 * V_follow_wall;

        % Normalize and update position
        V_direction = movement_speed * (V_direction / norm(V_direction));
        x_i(i) = x_i(i) + V_direction(1);
        y_i(i) = y_i(i) + V_direction(2);

        % Update the position of the marker directly
        set(plot_handles(i), 'XData', x_i(i), 'YData', y_i(i));
    end

    % Break if reached goal
    if norm([x_t - x_i(i), y_t - y_i(i)]) <= 0.5
        break
    end

    drawnow;
end


%% --- Function Definition ---

function V = move_to_goal_vector(x_i, y_i, x_t, y_t)
V = [x_t - x_i; y_t - y_i] ./ norm([x_t - x_i; y_t - y_i]);
end


function [x_o, y_o, d_0] = nearest_obstacle(x_i, y_i, obstacles)
% Calculate distances from each obstacle to robot
robot_position = [x_i, y_i];
distances = arrayfun(@(x, y, r) sqrt((x - robot_position(1))^2 + (y - robot_position(2))^2) - r, obstacles(:, 1), obstacles(:, 2), obstacles(:, 3));

% Find the nearest obstacle and its distance
[d_0, idx] = min(distances);
x_o = obstacles(idx, 1);
y_o = obstacles(idx, 2);
end


function V = avoid_obstacle_vector(x_i, y_i, x_o, y_o, V_move_to_goal)
robot_to_obstacle = [x_o - x_i; y_o - y_i];
sign_pm = sign(dot(V_move_to_goal, robot_to_obstacle, 1));
V = (1 / norm(robot_to_obstacle)) * [sign_pm * (y_o - y_i); -sign_pm * (x_o - x_i)];
end


function V = follow_wall_vector(x_i, y_i, x_o, y_o)
sign_pm = 1; % Change this based on the relationship between the robot's moving direction and the obstacles
V = (1 / sqrt((x_o - x_i).^2 + (y_o - y_i).^2)) * [sign_pm * (y_o - y_i); -sign_pm * (x_o - x_i)];
end


function f1 = control_parameter_f1(d_m, a_m, b_m)
if d_m > b_m
    f1 = a_m;
else
    f1 = a_m * (d_m / b_m);
end
end


function f2 = control_parameter_f2(d_0, a_0, b_0, b_f)
if d_0 > b_0 || d_0 < b_f
    f2 = 0;
else
    f2 = a_0 * (d_0 / (b_f - b_0) + b_0 / (b_0 - b_f));
end
end


function f3 = control_parameter_f3(a_f, V_avoid_obstacle, V_follow_wall)
% Implement wall-following conditions here
wall_following_on = true; % Change this based on the conditions for wall-following behavior

if wall_following_on
    f3 = a_f;
else
    f3 = 0;
end
end
