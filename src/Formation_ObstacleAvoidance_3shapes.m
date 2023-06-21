%% Communication-aware Formation Control
clear all;
close all;
clc;

% Initialize all parameters
max_iter    = 500;
h           = 1;
swarm_size  = 7;
alpha       = 10^(-5);                 % system parameter about antenna characteristics
delta       = 2;                       % required application data rate
beta        = alpha*(2^delta-1);
v           = 3;                       % path loss exponent
r0          = 5;                       % reference antenna near-field
PT          = 0.94;                    % reception probability threshold
rho_ij      = 0;
formation_speed = 1;
travel_speed = 1;
communication_qualities = zeros(swarm_size, swarm_size);
am         = 0.3; % was 0.3
bm         = 1;
a0         = 0.7; % was 0.7
b0         = 10; % was 7
bf         = 3;
af         = 1;


% The position of the destination
dest_x = 180;
dest_y = 130;

% The position of square obstacle
sq_x = 70;
sq_y = 40;
sq_side_length = 20;

% The position of circle obstacle
circle_x = 110;
circle_y = 70;
circle_center = [circle_x, circle_y];
circle_radius = 10;

% % The position of rectangle obstacle
rec_x = 140;
rec_y = 80;
rec_width = 10;
rec_height = 30;

obstacles = [
    struct('type', 'square', 'center', [sq_x, sq_y], 'sideLength', sq_side_length, 'radius', NaN, 'width', NaN, 'height', NaN),
    struct('type', 'circle', 'center', [circle_x, circle_y], 'radius', circle_radius, 'sideLength', NaN, 'width', NaN, 'height', NaN),
    struct('type', 'rectangle', 'center', [rec_x, rec_y], 'width', rec_width, 'height', rec_height, 'radius', NaN, 'sideLength', NaN)
    ];

obs_mat = [sq_x, sq_y; circle_x, circle_y; rec_x, rec_y];
obs_mat_size = size(obs_mat, 1);

swarm_obs = [];


%% ---Initialize Agents' Positions---
swarm = [
    7, 0;
    7, -20;
    0,   -10;
    35,  -20;
    58,   0;
    52,  13;
    52, -18;
    ];


%% ---Initialize the velocity---
for j = 1:swarm_size
    speed(j,1) = 0;
    speed(j,2) = 0;
    prev_speed(j,1) = 0;
    prev_speed(j,2) = 0;
end


%% ---Performance Indicators---
t_Elapsed = 0;
Jn        = 0;
rn        = 0;

% Define the figure positions
figure_positions = [
    %Left Bottom Right Width Height
    750, 480, 500, 400;    % Position for Figure 2
    200, 10, 500, 400;    % Position for Figure 3
    200, 480, 500, 400;   % Position for Figure 1
    750, 10, 500, 400;    % Position for Figure 4
    ];

% Plot Jn
figure(1)
Jn_Plot = plot(t_Elapsed, Jn);
set(gcf, 'Position', figure_positions(1, :));
xlabel('$t(s)$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
ylabel('$J_n$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
title('Average Communication Performance Indicator');
hold on
Jn_Text = text(t_Elapsed(end), Jn(end), sprintf('Jn: %.4f', Jn(end)), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

% Plot rn
figure(2)
rn_Plot = plot(t_Elapsed, rn);
set(gcf, 'Position', figure_positions(2, :));
xlabel('$t(s)$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
ylabel('$r_n$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
title('Average Distance Indicator');
hold on
rn_Text = text(t_Elapsed(end), rn(end), sprintf('rn: %.4f', rn(end)), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');

drawnow                                         % Force a graphics refresh so that isn't counted in our elapsed time

tic

% Assign a different color to each edge-label pair
line_colors = rand(swarm_size, swarm_size, 3);
label_colors = line_colors;

% Define the color matrix with predefined colors
node_colors = [
    108 155 207;  % Light Blue
    247 147 39;   % Orange
    242 102 171;  % Light Pink
    255 217 90;   % Light Gold
    122 168 116;  % Green
    147 132 209;  % Purple
    245 80 80     % Red
    ] / 255;      % Divide by 255 to scale the RGB values to the [0, 1] range

[img, map, alphachannel] = imread('drone','png');
markersize = [3, 3];


%% ---Simulation---
for k=1:max_iter

    % Plot the node trace inside the loop
    figure(4)
    set(gcf, 'Position', figure_positions(4, :));
    xlabel('$x$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
    ylabel('$y$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
    title('Node Trace');
    hold on;

    % Plot all nodes as markers
    scatter(swarm(:, 1), swarm(:, 2), [], node_colors, 'filled');

    %--- Formation Scene + Node Trace---
    for i = 1:swarm_size
        % Plot the node trace
        swarm_trace(k, i, :) = swarm(i, :);

        trace_x = squeeze(swarm_trace(:, i, 1));
        trace_y = squeeze(swarm_trace(:, i, 2));
        plot(trace_x, trace_y, 'Color', node_colors(i, :));

        % Add arrows to indicate node movement
        arrow_x = trace_x(1:end-1);
        arrow_y = trace_y(1:end-1);
        arrow_dx = diff(trace_x);
        arrow_dy = diff(trace_y);

        % Normalize the arrow displacements
        arrow_magnitudes = sqrt(arrow_dx.^2 + arrow_dy.^2);
        max_arrow_magnitude = max(arrow_magnitudes);
        scaling_factor = 2;  % Adjust the scaling factor for arrow size
        normalized_arrow_dx = arrow_dx * scaling_factor / max_arrow_magnitude;
        normalized_arrow_dy = arrow_dy * scaling_factor / max_arrow_magnitude;

        quiver(arrow_x, arrow_y, normalized_arrow_dx, normalized_arrow_dy, 0, 'Color', node_colors(i, :), 'LineWidth', 1.5, 'MaxHeadSize', 2);
    end

    figure(3);
    clf; % Clear the figure
    set(gcf, 'Position', figure_positions(3, :));
    xlabel('$x$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
    ylabel('$y$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
    title('Formation Scene');
    axis equal;

    % Plot the obstacle
    fill(sq_x + [-sq_side_length/2 sq_side_length/2 sq_side_length/2 -sq_side_length/2 -sq_side_length/2], sq_y + [-sq_side_length/2 -sq_side_length/2 sq_side_length/2 sq_side_length/2 -sq_side_length/2], 'r');
    hold on;

    % Plot the rectangle
    rectangle('Position', [rec_x, rec_y, rec_width, rec_height], 'FaceColor', 'r');
    hold on;

    % Plot the circle
    theta = linspace(0, 2*pi, 100);
    circle_x = circle_center(1) + circle_radius * cos(theta);
    circle_y = circle_center(2) + circle_radius * sin(theta);
    fill(circle_x, circle_y, 'r');
    hold on;

    % Plot the destination
    fill([dest_x - 2, dest_x + 2, dest_x + 2, dest_x - 2, dest_x - 2], [dest_y - 2, dest_y - 2, dest_y + 2, dest_y + 2, dest_y - 2], 'k');
    text(dest_x + 5, dest_y, 'Destination', 'Color', 'k', 'FontSize', 12, 'HorizontalAlignment', 'left');
    hold on;
    if size(swarm_obs, 1)
        plot(swarm_obs(:, 1), swarm_obs(:, 2), 'ro');
    end

    for l = 1:swarm_size
        x_low = swarm(l, 1) - markersize(1)/2;
        x_high = swarm(l, 1) + markersize(1)/2;
        y_low = swarm(l, 2) - markersize(2)/2;
        y_high = swarm(l, 2) + markersize(2)/2;
        imagesc([x_low x_high], [y_low y_high], img, 'AlphaData', alphachannel, 'CData', repmat(reshape(node_colors(l, :), [1 1 3]), [size(img, 1), size(img, 2), 1]));
    end


    %% --- Plot Formation Edges + Labels ---
    for i = 1:swarm_size
        for j= 1:swarm_size
            if i ~= j && communication_qualities(i, j) > PT
                hold on;
                % Get the line color and label color for the current pair
                line_color = line_colors(i, j, :);
                label_color = label_colors(i, j, :);

                a1 = swarm(i,:);
                a2 = swarm(j,:);
                plot([a1(1), a2(1)], [a1(2), a2(2)], '--', 'Color', line_color);


                % Calculate the position for the label
                scaling_factor = 0.2;  % Adjust the scaling factor for the label positioning
                midpoint_x = (a1(1) + a2(1)) / 2;
                midpoint_y = (a1(2) + a2(2)) / 2;
                displacement_x = (a2(1) - a1(1)) * scaling_factor;
                displacement_y = (a2(2) - a1(2)) * scaling_factor;
                label_x = midpoint_x + displacement_x;
                label_y = midpoint_y + displacement_y + 2;

                % Add the number label at the midpoint of the line
                label = communication_qualities(i, j);
                label_str = sprintf('%.4f', label);
                text(label_x, label_y, label_str, 'HorizontalAlignment', 'center', 'Color', label_color);
                hold off;

                % Remove quality value for refresh
                communication_qualities(i, j) = 0;
                communication_qualities(j, i) = 0;
            end
        end
    end

    hold off;
    axis equal;

    %% --- Controller ---
    for i=1:swarm_size
        rho_ij=0;
        phi_rij=0;
        rij=0;
        for j=[1:(i-1),(i+1):swarm_size]
            rij=sqrt((swarm(i,1)-swarm(j,1))^2+(swarm(i,2)-swarm(j,2))^2);
            aij=exp(-alpha*(2^delta-1)*(rij/r0)^v);
            gij=rij/sqrt(rij^2+r0^2);

            if aij>=PT
                rho_ij=(-beta*v*rij^(v+2)-beta*v*(r0^2)*(rij^v)+r0^(v+2))*exp(-beta*(rij/r0)^v)/sqrt((rij^2+r0^2)^3);
            else
                rho_ij=0;
            end

            phi_rij=gij*aij;
            communication_qualities(i,j) = phi_rij;
            qi=[swarm(i,1),swarm(i,2)];
            qj=[swarm(j,1),swarm(j,2)];
            nd=(qi-qj)/sqrt(1+norm(qi-qj)*formation_speed) * travel_speed;

            speed(i,1)=speed(i,1)+rho_ij * nd(1);
            speed(i,2)=speed(i,2)+rho_ij * nd(2);

            % fprintf("Time: %d\n", trigger);
            if k >= 40
                % Reduce communication quality when jammed
                for i = 1:size(swarm, 1)
                    updateCommunicationParametersWithObstacles(swarm(i, 1), swarm(i, 2), aij, gij, obstacles);
                end

                %---Senario 1: Reach to Goal---
                fprintf("Stage 1: Reaching Goal Starts\n")
                destination_vector = [dest_x - swarm(i, 1), dest_y - swarm(i, 2)];
                dist_vector = norm(destination_vector);
                destination_speed = destination_vector / dist_vector;

                if dist_vector > bm
                    contr_param = am;
                else
                    contr_param = am * (dist_vector/bm);
                end

                speed(i, 1) = speed(i, 1) + (destination_speed(1) * contr_param);
                speed(i, 2) = speed(i, 2) + (destination_speed(2) * contr_param);
                fprintf("Stage 1: Reaching Goal Ends\n")


                %---Senario 2: Obstacle Avoidance---
                fprintf("Stage 2: Obstacle Avoidance Start\n")
                if size(swarm_obs) > 0
                    for l=1:obs_mat_size
                        swarm_obs_x = obs_mat(l, 1);
                        swarm_obs_y = obs_mat(l, 2);
                        avoidance_vec = [swarm_obs_y - swarm(i, 2), -(swarm_obs_x - swarm(i, 1))];
                        obs_dist = norm(avoidance_vec);
                    end

                    if avoidance_vec(1) >= 0 % Obstacle above Agent
                        avoidance_vec(1) = -avoidance_vec(1);
                    else % Obstacle below Agent
                        avoidance_vec(1) =  avoidance_vec(1);
                    end

                    %                         if avoidance_vec(2) >= 0 % Obstacle on the right of Agent
                    %                             avoidance_vec(2) = -avoidance_vec(2);
                    %                         else % Obstacle on the left of Agent
                    %                             avoidance_vec(2) = avoidance_vec(2);
                    %                         end

                    avoidance_speed = avoidance_vec / obs_dist;

                    if obs_dist >= bf && obs_dist <= b0
                        contr_param = a0 * ( obs_dist/(bf-b0) + b0/(b0-bf) );
                    else
                        contr_param = 0;
                    end

                    % Update the speed with obstacle avoidance
                    speed(i, 1) = speed(i, 1) + avoidance_speed(2) * contr_param;
                    speed(i, 2) = speed(i, 2) + avoidance_speed(1) * contr_param;
                    fprintf("Stage 2: Obstacle Avoidance Ends\n")


                    %---Senario 3: Wall Following---
                    fprintf("Stage 3: Wall Following Starts\n")
                    if obs_dist < 5
                        % fprintf("Wall following\n")
                        wall_follow = af;
                    else
                        wall_follow = 0;
                    end

                    % Update the speed with wall following
                    speed(i, :) = speed(i, :) + avoidance_speed * contr_param;
                    fprintf("Stage 3: Wall Following Starts\n")
                end
            end
        end


        if all(communication_qualities(i, :) < PT)
            if ~ismember(swarm(i, :), swarm_obs)
                swarm_obs = [swarm_obs; swarm(i, :)];
                fprintf("Obstacle found, new size: %d\n", size(swarm_obs, 1));
            end

            swarm(i,1)=swarm(i,1)-prev_speed(i,1) * 1.5;
            swarm(i,2)=swarm(i,2)-prev_speed(i,2) * 1.5;
        else
            swarm(i,1)=swarm(i,1)+speed(i,1)*h;
            swarm(i,2)=swarm(i,2)+speed(i,2)*h;
        end

        prev_speed(i,1) = speed(i,1)*h;
        prev_speed(i,2) = speed(i,2)*h;

        speed(i,1)=0;
        speed(i,2)=0;

        t_Elapsed=cat(1, t_Elapsed, toc);

        %---Average Communication Performance Indicator---%
        Jn=cat(1, Jn, phi_rij);
        Jn=smooth(Jn);
        set(Jn_Plot, 'xdata', t_Elapsed, 'ydata', Jn);      % Plot Jn
        set(Jn_Text, 'Position', [t_Elapsed(end), Jn(end)], 'String', sprintf('Jn: %.4f', Jn(end)));
        pause(0);

        %---Average Neighboring Distance Indicator---%
        rn=cat(1, rn, rij);
        rn=smooth(rn);
        set(rn_Plot, 'xdata', t_Elapsed, 'ydata', rn);      % Plot rn
        set(rn_Text, 'Position', [t_Elapsed(end), rn(end)], 'String', sprintf('rn: %.4f', rn(end)));

        pause(0);

    end

    pause(0)

end

figure(1)
axis([0 max_iter+10  min(Jn) max(Jn)+5]);  % Update the limits for Figure 1

figure(2)
axis([0 max_iter+10 min(rn) max(rn)+5]);  % Update the limits for Figure 2

% Plot the final node trace outside the loop
figure(4)
hold on;
for i = 1:swarm_size
    trace_x = squeeze(swarm_trace(:, i, 1));
    trace_y = squeeze(swarm_trace(:, i, 2));
    plot(trace_x, trace_y);
end
axis([x_min x_max y_min y_max]);
hold off;