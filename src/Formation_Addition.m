
%% ---Initialize Agents' Positions---
swarm = [-5,  14;  
         -5, -19;   
          0,   0;   
         20,  20;
         35,  -4;   
         68,   0;  
         72,  13;];

% Create a new_agent matrix
new_agent = [50, 50];

% Append new_agent to the swarm
swarm = [swarm; new_agent];


%% Define the range of coordinates
x_min = 0;
x_max = 50;
y_min = 0;
y_max = 50;

% Generate random positions for the swarm
x_coords = x_min + (x_max - x_min) * rand(swarm_size, 1);
y_coords = y_min + (y_max - y_min) * rand(swarm_size, 1);

% Combine the x and y coordinates into a single matrix
swarm = [x_coords, y_coords];

% Define new_agent coordinates
new_x = 50;
new_y = 50;

% Create a new_agent matrix
new_agent = [new_x, new_y];

% Append new_agent to the swarm
swarm = [swarm; new_agent];

% Print agents' initial positions
for i = 1:swarm_size
    for j = 1:2
        fprintf("%d", swarm(i, j))
    end
    fprintf("\n")
end
