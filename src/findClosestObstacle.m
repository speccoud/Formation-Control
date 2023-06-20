function closestCoordinates = findClosestObstacle(targetCoordinates, swarm_obs)
    closestDistance = inf;
    closestIndex = 0;

    for i = 1:size(swarm_obs, 1)  % Iterate over the rows of swarm_obs
        coordinates = swarm_obs(i, :);

        distance = sqrt(sum((targetCoordinates - coordinates).^2));

        if distance < closestDistance
            closestDistance = distance;
            closestIndex = i;
        end
    end

    closestCoordinates = swarm_obs(closestIndex, :);
end
