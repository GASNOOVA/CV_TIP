function animateTIP(fig, img, rectangle_x, rectangle_y, vanishing_point_x, vanishing_point_y)

    % TIP basic algorithmus
    % Vanishing points analysis
    % Version: 1.1
    % Last change: 25.06.2024
    % Author: Cheng Chen
    
    end_x = size(img, 2);
    end_y = size(img, 1);
    
    % Calculating the vectors
    vector_top_left = [rectangle_x(1)-vanishing_point_x; rectangle_y(1)-vanishing_point_y];
    vector_top_right = [rectangle_x(2)-vanishing_point_x; rectangle_y(2)-vanishing_point_y];
    
    vector_bottom_right = [rectangle_x(3)-vanishing_point_x; rectangle_y(3)-vanishing_point_y];
    vector_bottom_left = [rectangle_x(4)-vanishing_point_x; rectangle_y(4)-vanishing_point_y];
    
    vanishing_point = [vanishing_point_x; vanishing_point_y];
    
    intersection_tl_left = intersection_with_image_border(vanishing_point, vector_top_left, [0; 0], [0; 1]);
    intersection_tl_top = intersection_with_image_border(vanishing_point, vector_top_left, [0; 0], [1; 0]);
    
    intersection_tr_right = intersection_with_image_border(vanishing_point, vector_top_right, [end_x; 0], [0; 1]);
    intersection_tr_top = intersection_with_image_border(vanishing_point, vector_top_right, [end_x; 0], [1; 0]);
    
    intersection_br_right = intersection_with_image_border(vanishing_point, vector_bottom_right, [end_x; end_y], [0; 1]);
    intersection_br_bottom = intersection_with_image_border(vanishing_point, vector_bottom_right, [end_x; end_y], [1; 0]);
    
    intersection_bl_left = intersection_with_image_border(vanishing_point, vector_bottom_left, [0; end_y], [0; 1]);
    intersection_bl_bottom = intersection_with_image_border(vanishing_point, vector_bottom_left, [0; end_y], [1; 0]);
    
    intersections = [intersection_tl_left, intersection_tl_top, intersection_tr_right, intersection_tr_top, intersection_br_right, intersection_br_bottom, intersection_bl_left, intersection_bl_bottom];
    
    plot(intersections(1,:), intersections(2,:), 'r*', 'LineWidth', 10); % Display the selected corners
    hold on;
    imshow(img)
    
    threed_rectangle_bottom_left = [rectangle_x(4); 0; rectangle_y(4)];
    threed_rectangle_bottom_right = [rectangle_x(3); 0; rectangle_y(3)];
    
    height = minDistance([rectangle_x(1); rectangle_y(1)], [rectangle_x(4); rectangle_y(4)], [rectangle_x(2); rectangle_y(2)], [rectangle_x(3); rectangle_y(3)]);
    
    threed_rectangle_top_left = [threed_rectangle_bottom_left(1); height; threed_rectangle_bottom_left(3)];
    threed_rectangle_top_right = [threed_rectangle_bottom_right(1); height; threed_rectangle_bottom_right(3)];
    
    distance_bottom = minDistance([intersection_br_bottom(1); intersection_br_bottom(2)], [rectangle_x(3); rectangle_y(3)], [intersection_bl_bottom(1); intersection_bl_bottom(2)], [rectangle_x(4); rectangle_y(4)]);
    distance_top = minDistance([intersection_tr_top(1); intersection_tr_top(2)], [rectangle_x(2); rectangle_y(2)], [intersection_tl_top(1); intersection_tl_top(2)], [rectangle_x(1); rectangle_y(1)]);
    distance_left = minDistance([intersection_tl_left(1); intersection_tl_left(2)], [rectangle_x(1); rectangle_y(1)],[intersection_bl_left(1); intersection_bl_left(2)], [rectangle_x(4); rectangle_y(4)]);
    distance_right = minDistance([intersection_tr_right(1); intersection_tr_right(2)], [rectangle_x(2); rectangle_y(2)],[intersection_br_right(1); intersection_br_right(2)], [rectangle_x(3); rectangle_y(3)]);
    
    threed_intersection_tl_left = ninetyDegreePoint(threed_rectangle_top_left, threed_rectangle_top_right-threed_rectangle_top_left, distance_left);
    threed_intersection_bl_left = ninetyDegreePoint(threed_rectangle_bottom_left, threed_rectangle_top_right-threed_rectangle_top_left, distance_left);
    
    threed_intersection_tr_right = ninetyDegreePoint(threed_rectangle_top_right, threed_rectangle_top_right-threed_rectangle_top_left, distance_right);
    threed_intersection_br_right = ninetyDegreePoint(threed_rectangle_bottom_right, threed_rectangle_top_right-threed_rectangle_top_left, distance_right);
    
    threed_intersection_tl_top = ninetyDegreePoint(threed_rectangle_top_left, threed_rectangle_top_right-threed_rectangle_top_left, distance_top);
    threed_intersection_tr_top = ninetyDegreePoint(threed_rectangle_top_right, threed_rectangle_top_right-threed_rectangle_top_left, distance_top);
    
    threed_intersection_bl_bottom = ninetyDegreePoint(threed_rectangle_bottom_left, threed_rectangle_top_right-threed_rectangle_top_left, distance_bottom);
    threed_intersection_br_bottom = ninetyDegreePoint(threed_rectangle_bottom_right, threed_rectangle_top_right-threed_rectangle_top_left, distance_bottom);
    
    
    threed_points = [threed_rectangle_bottom_right, threed_rectangle_bottom_left, threed_rectangle_top_right, threed_rectangle_top_left, threed_intersection_br_right, threed_intersection_br_bottom, threed_intersection_bl_left, threed_intersection_bl_bottom, threed_intersection_tl_left, threed_intersection_tl_top, threed_intersection_tr_right, threed_intersection_tr_top];
    
    displayTransformedSegments(fig, img, threed_points, rectangle_x, rectangle_y, intersections);
end

function intersection_with_border = intersection_with_image_border(vanishing_point, direction, border_point, border_direction)
    % Construct the system of equations
    A = [direction(:), -border_direction(:)];
    b = border_point(:) - vanishing_point(:);
    
    % Solve the system of linear equations
    t = A \ b;
    
    % Calculate the intersection point
    intersection_with_border = vanishing_point + t(1) * direction;
end

function [model, inlierIdx] = fitLineRANSAC(points, tolerance)
    % Fit line using RANSAC to better handle outliers
    [model, inlierIdx] = ransac(points, @(x) polyfit(x(:,1), x(:,2), 1), ...
        @(model, points) abs(polyval(model, points(:,1)) - points(:,2)) < tolerance, ...
        size(points,1), 0.5 * size(points, 1));  % Adjust iterations and sample count as needed
end

function edgeDensity = calculateEdgeDensity(edges, windowSize)
    % Calculate the local density of edge pixels within a window
    % Create a window with all ones
    window = ones(windowSize, windowSize);
    
    % Use convolution to count the number of edge pixels in the neighborhood
    edgeCounts = conv2(double(edges), window, 'same');
    
    % Normalize by the area of the window to get density
    edgeDensity = edgeCounts / (windowSize * windowSize);
end

function [min_distance] = minDistance(v1, r1, v2, r2)
    min_distance=min(norm(v1-r1), norm(v2-r2))
end

function [point] = ninetyDegreePoint(startingPoint, originalDirection, distance)
    unitOriginalDirection = originalDirection / norm(originalDirection)
    newDirection = [-unitOriginalDirection(3); unitOriginalDirection(2); unitOriginalDirection(1)] * distance
    point = startingPoint + newDirection
end
