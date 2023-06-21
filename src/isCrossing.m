function isCrossing = isCrossing(lineStart, lineEnd, obstacle)
    % Extract the coordinates of the line and obstacle
    x1 = lineStart(1);
    y1 = lineStart(2);
    x2 = lineEnd(1);
    y2 = lineEnd(2);

    % Check the type of obstacle and handle accordingly
    switch obstacle.type
        case 'square'
            isCrossing = isCrossingSquare(x1, y1, x2, y2, obstacle.center, obstacle.sideLength);
        case 'circle'
            isCrossing = isCrossingCircle(x1, y1, x2, y2, obstacle.center, obstacle.radius);
        case 'rectangle'
            isCrossing = isCrossingRectangle(x1, y1, x2, y2, obstacle.center, obstacle.width, obstacle.height);
        otherwise
            error('Unsupported obstacle type');
    end
end

function isCrossing = isCrossingSquare(x1, y1, x2, y2, squareCenter, sideLength)
    % Extract the coordinates of the line and square
    x1 = lineStart(1);
    y1 = lineStart(2);
    x2 = lineEnd(1);
    y2 = lineEnd(2);
    squareX = squareCenter(1) - sideLength/2;
    squareY = squareCenter(2) - sideLength/2;
    
    % Check if any of the line endpoints are inside the square
    if (x1 >= squareX && x1 <= squareX + sideLength && y1 >= squareY && y1 <= squareY + sideLength) || ...
       (x2 >= squareX && x2 <= squareX + sideLength && y2 >= squareY && y2 <= squareY + sideLength)
        isCrossing = true;
        return;
    end
    
    % Check if any of the line segments intersect with the square
    if doLineSegmentsIntersect(x1, y1, x2, y2, squareX, squareY, squareX + sideLength, squareY) || ...
       doLineSegmentsIntersect(x1, y1, x2, y2, squareX + sideLength, squareY, squareX + sideLength, squareY + sideLength) || ...
       doLineSegmentsIntersect(x1, y1, x2, y2, squareX + sideLength, squareY + sideLength, squareX, squareY + sideLength) || ...
       doLineSegmentsIntersect(x1, y1, x2, y2, squareX, squareY + sideLength, squareX, squareY)
        isCrossing = true;
        return;
    end
    
    % If none of the above conditions are met, the line does not cross the square
    isCrossing = false;
end

function isIntersecting = doLineSegmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4)
    % Determine the orientation of three points
    orientation1 = orientation(x1, y1, x2, y2, x3, y3);
    orientation2 = orientation(x1, y1, x2, y2, x4, y4);
    orientation3 = orientation(x3, y3, x4, y4, x1, y1);
    orientation4 = orientation(x3, y3, x4, y4, x2, y2);
    
    % Check if the line segments intersect
    if orientation1 ~= orientation2 && orientation3 ~= orientation4
        isIntersecting = true;
    elseif orientation1 == 0 && isPointOnSegment(x1, y1, x2, y2, x3, y3)
        isIntersecting = true;
    elseif orientation2 == 0 && isPointOnSegment(x1, y1, x2, y2, x4, y4)
        isIntersecting = true;
    elseif orientation3 == 0 && isPointOnSegment(x3, y3, x4, y4, x1, y1)
        isIntersecting = true;
    elseif orientation4 == 0 && isPointOnSegment(x3, y3, x4, y4, x2, y2)
        isIntersecting = true;
    else
        isIntersecting = false;
    end
end

function orient = orientation(x1, y1, x2, y2, x3, y3)
    value = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
    if value > 0
        orient = 1; % Counter-clockwise orientation
    elseif value < 0
        orient = -1; % Clockwise orientation
    else
        orient = 0; % Collinear points
    end
end

function isOnSegment = isPointOnSegment(x1, y1, x2, y2, x3, y3)
    if x3 >= min(x1, x2) && x3 <= max(x1, x2) && y3 >= min(y1, y2) && y3 <= max(y1, y2)
        isOnSegment = true;
    else
        isOnSegment = false;
    end
end


function isCrossing = isCrossingCircle(x1, y1, x2, y2, circleCenter, radius)
    % Calculate the direction vector of the line segment
    dx = x2 - x1;
    dy = y2 - y1;
    
    % Calculate the vector between the line start and circle center
    cx = circleCenter(1);
    cy = circleCenter(2);
    v = [cx - x1, cy - y1];
    
    % Calculate the quadratic coefficients
    a = dx^2 + dy^2;
    b = 2 * (dx * v(1) + dy * v(2));
    c = v(1)^2 + v(2)^2 - radius^2;
    
    % Check if the line segment intersects the circle
    discriminant = b^2 - 4 * a * c;
    if discriminant < 0
        isCrossing = false;
    else
        t1 = (-b + sqrt(discriminant)) / (2 * a);
        t2 = (-b - sqrt(discriminant)) / (2 * a);
        isCrossing = (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
    end
end


function isCrossing = isCrossingRectangle(x1, y1, x2, y2, rectCenter, width, height)
    % Calculate the half width and half height
    halfWidth = width / 2;
    halfHeight = height / 2;
    
    % Calculate the coordinates of the rectangle's corners
    x = rectCenter(1);
    y = rectCenter(2);
    corners = [x - halfWidth, y - halfHeight;   % Top-left corner
               x + halfWidth, y - halfHeight;   % Top-right corner
               x + halfWidth, y + halfHeight;   % Bottom-right corner
               x - halfWidth, y + halfHeight];  % Bottom-left corner
    
    % Check if any of the line segments intersect with the rectangle
    isCrossing = false;
    for i = 1:4
        j = mod(i, 4) + 1;
        if doLineSegmentsIntersect(x1, y1, x2, y2, corners(i, 1), corners(i, 2), corners(j, 1), corners(j, 2))
            isCrossing = true;
            break;
        end
    end
end

% ... existing code for doLineSegmentsIntersect, orientation, and isPointOnSegment
