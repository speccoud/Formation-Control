%% --- Main ---
close all;
clear all;
clc;

% Robot position
robot_pos = [
    7, 0;
    7, -20;
    0, -10;
    35, -20;
    58, 0;
    52, 13;
    52, -18
    ];

% The position of the goal
goal_x = 190;
goal_y = 140;
goal_pos = [goal_x, goal_y];

% The position of square obstacle
sq_x = 70;
sq_y = 40;
sq_side_length = 20;

% The position of circle obstacle
circle_x = 110;
circle_y = 70;
circle_center = [circle_x, circle_y];
circle_radius = 10;

% The position of rectangle obstacle
rec_x = 150;
rec_y = 100;
rec_width = 10;
rec_height = 30;

% Obstacles
obstacles = [
    struct('type', 'square', 'center', [sq_x, sq_y], 'sideLength', sq_side_length, 'radius', NaN, 'width', NaN, 'height', NaN),
    struct('type', 'circle', 'center', [circle_x, circle_y], 'radius', circle_radius, 'sideLength', NaN, 'width', NaN, 'height', NaN),
    struct('type', 'rectangle', 'center', [rec_x, rec_y], 'width', rec_width, 'height', rec_height, 'radius', NaN, 'sideLength', NaN)
    ];

% Simulation parameters
num_robots = size(robot_pos, 1);
num_iterations = 1000;
dt = 10;

% Weight factors and constants
a_m = 1; % Movement behavior parameter
b_m = 1; % Movement behavior parameter
a_0 = 1; % Obstacle avoidance behavior parameter
b_0 = 10; % Obstacle avoidance behavior parameter
b_f = 3; % Wall-following behavior parameter
a_f = 1; % Wall-following behavior parameter

k_goal = 1;
max_velocity = 10;
k_obstacle = 1;
max_range = 10;
k_wall = 1;
follow_wall_distance = 5;

% Preallocate arrays for storing robot positions
robot_positions = zeros(num_iterations, num_robots, 2);
robot_positions(1, :, :) = robot_pos;

% Animation setup
figure;
hold on;
axis equal;

% Plot goal position
plot(goal_pos(1), goal_pos(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2);

% Plot obstacles
for i = 1:numel(obstacles)
    obstacle = obstacles(i);
    if strcmp(obstacle.type, 'square')
        x = obstacle.center(1) - obstacle.sideLength/2;
        y = obstacle.center(2) - obstacle.sideLength/2;
        w = obstacle.sideLength;
        h = obstacle.sideLength;
        rectangle('Position', [x, y, w, h], 'FaceColor', 'r');
    elseif strcmp(obstacle.type, 'circle')
        x = obstacle.center(1) - obstacle.radius;
        y = obstacle.center(2) - obstacle.radius;
        d0 = 2 * obstacle.radius;
        rectangle('Position', [x, y, d0, d0], 'Curvature', [1, 1], 'FaceColor', 'r');
    elseif strcmp(obstacle.type, 'rectangle')
        x = obstacle.center(1) - obstacle.width/2;
        y = obstacle.center(2) - obstacle.height/2;
        w = obstacle.width;
        h = obstacle.height;
        rectangle('Position', [x, y, w, h], 'FaceColor', 'r');
    end
end

% Plot initial robot positions
for i = 1:num_robots
    x = robot_positions(1, i, 1);
    y = robot_positions(1, i, 2);
    plot(x, y, 'b.', 'MarkerSize', 10);
end

% Animation loop
for iter = 2:num_iterations
    for i = 1:num_robots
        % Calculate the combined force for each robot
        F_total = combinedForce(robot_positions(iter-1, i, :), goal_pos, obstacles, a_m, b_m, a_0, b_0, b_f, a_f);

        % Update the robot's position based on the combined force
        robot_positions(iter, i, :) = updatePosition(robot_positions(iter-1, i, :), F_total, max_velocity, dt * i); % <-- Multiply dt by i

        % Plot updated robot positions
        x = robot_positions(iter, i, 1);
        y = robot_positions(iter, i, 2);
        plot(x, y, 'b.', 'MarkerSize', 10);
    end

    drawnow; % Update the figure
    pause(1); % Pause for a short duration to create animation effect
end


hold off;


%% --- Function Definition ---

function F_total = combinedForce(robot_pos, goal_pos, obstacles, a_m, b_m, a_0, b_0, b_f, a_f)
    % Calculate forces
    F_goal = moveTowardsGoal(robot_pos, goal_pos, a_m, b_m);
    F_avoid_obstacle = avoidObstacles(robot_pos, obstacles, a_0, b_0, goal_pos); 
    F_follow_wall = followWall(robot_pos, goal_pos, a_f, b_f);

    % Sum all forces
    F_total = F_goal + F_avoid_obstacle + F_follow_wall;
end



function F_move_to_goal = moveTowardsGoal(robot_pos, goal_pos, a_m, b_m)
% Reshape the arrays if needed
robot_pos = reshape(robot_pos, 1, []);
goal_pos = reshape(goal_pos, 1, []);

% Calculate distance between robot and goal
d_m = sqrt(sum((goal_pos - robot_pos).^2));

% Calculate behavior parameter
if d_m >= b_m
    f1 = a_m;
else
    f1 = a_m * (d_m / b_m);
end

% Calculate behavior vector
V_move_to_goal = (goal_pos - robot_pos) / norm(goal_pos - robot_pos);

% Calculate force
F_move_to_goal = f1 * V_move_to_goal;
end


function F_avoid_obstacle = avoidObstacles(robot_pos, obstacles, a_0, b_0, goal_pos)
    % Initialize behavior vector
    V_avoid_obstacle = [0, 0];

    % Reshape the arrays if needed
    robot_pos = reshape(robot_pos, 1, []);
    goal_pos = reshape(goal_pos, 1, []);

    for i = 1:numel(obstacles)
        obstacle = obstacles(i);

        % Calculate distance between robot and obstacle manually
        dx = obstacle.center(1) - robot_pos(1);
        dy = obstacle.center(2) - robot_pos(2);
        d_0 = sqrt(dx^2 + dy^2);

        % Check if obstacle is within range
        if d_0 <= b_0
            % Calculate behavior parameter
            f2 = a_0 * ((d_0 / (b_0 - b_f)) + (b_0 / (b_0 - b_f)));

            % Calculate behavior vector
            V_avoid_obstacle = V_avoid_obstacle + f2 * [
                -dy;
                dx
                ];
        end
    end

    % Calculate force
    F_avoid_obstacle = V_avoid_obstacle / norm(V_avoid_obstacle);

    % Adjust the behavior vector based on the direction towards the goal
    goal_vector = goal_pos - robot_pos;
    goal_direction = goal_vector / norm(goal_vector);
    F_avoid_obstacle = F_avoid_obstacle + goal_direction;
    F_avoid_obstacle = F_avoid_obstacle / norm(F_avoid_obstacle);
end


function F_follow_wall = followWall(robot_pos, obstacles, b_f, a_f)
    % Initialize behavior vector
    V_follow_wall = [0, 0];

    for i = 1:length(obstacles)
        obstacle = obstacles(i);
        % Calculate distance between robot and obstacle manually
        dx = obstacle.center(1) - robot_pos(1);
        dy = obstacle.center(2) - robot_pos(2);
        d_0 = sqrt(dx^2 + dy^2);

        % Check if obstacle is within range
        if d_0 <= b_f
            % Calculate behavior parameter
            f3 = a_f * ((b_f / (b_f - a_f)) - (d_0 / (b_f - a_f)));

            % Calculate behavior vector
            V_follow_wall = V_follow_wall + f3 * [
                -(obstacle.center(2) - robot_pos(2));
                obstacle.center(1) - robot_pos(1)
                ];
        end
    end

    % Calculate force
    F_follow_wall = V_follow_wall / norm(V_follow_wall);
end



function new_pos = updatePosition(old_pos, force, max_velocity, dt)
    % Reshape the arrays if needed
    old_pos = reshape(old_pos, 1, []);
    force = reshape(force, 1, []);

    % Calculate velocity
    velocity = force * max_velocity;

    % Calculate new position
    new_pos = old_pos + velocity * dt;
end


