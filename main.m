% Computer Vision Challenge
% Group 35
% Tour into the Picture
% ver 2.0
% update 09.07.2024

%% main GUI
function main
    % Create a figure for the GUI
    fig = uifigure('Name', 'Tour Into the Picture', 'Position', [100, 100, 800, 600]);

    % Load image button
    uibutton(fig, 'Position', [10, 570, 100, 20], 'Text', 'Load Image', 'ButtonPushedFcn', @(btn, event) loadImage(fig));
    
    % Extruct foreground objects
    uibutton(fig, 'Position', [120, 570, 150, 20], 'Text', 'Select Foreground', 'ButtonPushedFcn', @(btn, event) selectForeground(fig));
    
    % Select vanishing point button
    uibutton(fig, 'Position', [280, 570, 150, 20], 'Text', 'Select Vanishingpoint', 'ButtonPushedFcn', @(btn, event) selectVanishingPoint(fig));

    % Draw inner rectangle button
    uibutton(fig, 'Position', [440, 570, 150, 20], 'Text', 'Select rear Wall', 'ButtonPushedFcn', @(btn, event) drawInnerRectangle(fig));
    
    % Add button to project foreground
    uibutton(fig, 'Position', [600, 540, 150, 20], 'Text', 'Project Foreground', 'ButtonPushedFcn', @(btn, event) projectForeground(fig));
    
    % Start animation button (bottom right corner)
    uibutton(fig, 'Position', [600, 570, 190, 20], 'Text', 'Tour into the Picture!', 'ButtonPushedFcn', @(btn, event) startAnimation(fig));

    % Axes for displaying the image and animation
    axBackground = uiaxes(fig, 'Position', [10, 10, 540, 550]); % Adjusted size to accommodate the button
    setappdata(fig, 'BackgroundAxes', axBackground);

    % Axes for displaying the foreground image
    axForeground = uiaxes(fig, 'Position', [560, 175, 230, 250]);
    setappdata(fig, 'ForegroundAxes', axForeground);

    % Initialize data
    setappdata(fig, 'Image', []);
    setappdata(fig, 'VanishingPoint', []);
    setappdata(fig, 'InnerRectangle', []);
    setappdata(fig, 'VanishingPointHandle', []); % Store handle for vanishing point
    setappdata(fig, 'MeshLines', []); % Store handles for mesh lines
    setappdata(fig, 'RectangleHandle', []); % Store handle for inner rectangle
end

%% 1.Load Image
function loadImage(fig)
    [file, path] = uigetfile({'*.jpg;*.png'}, 'Select an Image');
    if isequal(file, 0)
        return;
    end
    img = imread(fullfile(path, file));
    setappdata(fig, 'Image', img);
    axBackground = getappdata(fig, 'BackgroundAxes');
    imshow(img, 'Parent', axBackground);

    % Adjust the axis properties to fit the image
    axis(axBackground, 'image');
    axBackground.XTick = [];
    axBackground.YTick = [];
end

%% 2.Separate the Foreground and Background
function selectForeground(fig)
    axBackground = getappdata(fig, 'BackgroundAxes');
    axForeground = getappdata(fig, 'ForegroundAxes');
    img = getappdata(fig, 'Image');
    masks = getappdata(fig, 'Masks');

    if isempty(img)
        uialert(fig, 'Load an image first!', 'Image Not Found');
        return;
    end

    % Initialize with the original image
    compositeBackground = img;
    % Initialize the foreground image as fully transparent (all zeros)
    foregroundImage = zeros(size(img), 'like', img);

    % 检查foregroundPositions是否已初始化
    if isappdata(fig, 'ForegroundPositions')
        foregroundPositions = getappdata(fig, 'ForegroundPositions');
    else
        foregroundPositions = [];  % 初始化为空矩阵
        setappdata(fig, 'ForegroundPositions', foregroundPositions);
    end
    
    finished = false;
    while ~finished
        h = drawpolygon('Parent', axBackground, 'Color', 'b');

        % Wait for user confirmation
        wait(h);
        
        if isvalid(h)
            % Create mask from polygon
            mask = createMask(h, img);
            masks{end+1} = mask;

            % Update composite foreground image
            currentForeground = img;
            currentForeground(repmat(~mask, [1, 1, 3])) = 0;
            foregroundImage = max(foregroundImage, currentForeground);

            % Update background
            for k = 1:3  % Apply regionfill to each channel
                compositeBackground(:,:,k) = regionfill(compositeBackground(:,:,k), mask);
            end

            % Calculate Foreground Position
            props = regionprops(mask, 'Centroid');
            foregroundPositions(end+1, :) = props.Centroid;

            % Display updated foreground and background
            imshow(foregroundImage, 'Parent', axForeground);
            imshow(compositeBackground, 'Parent', axBackground);
        else
            uialert(fig, 'Polygon was deleted or is invalid.', 'Error');
        end

        % Ask user if they want to add another foreground
        choice = uiconfirm(fig, 'Do you want to add another foreground?', 'Add Foreground', ...
                           'Options', {'Yes', 'No'}, 'DefaultOption', 2, 'CancelOption', 2);
        finished = strcmp(choice, 'No');
    end

    % Store the updated images and masks
    setappdata(fig, 'Image', compositeBackground);
    setappdata(fig, 'Masks', masks);
    setappdata(fig, 'ForegroundImage', foregroundImage);
    setappdata(fig, 'ForegroundPositions', foregroundPositions);  % 存储更新的前景位置信息
end

%% 3.Determine the Vanishing Point and Inner Rectangle
% 3.1 Determine Vanishing Point
function selectVanishingPoint(fig)
    ax = getappdata(fig, 'BackgroundAxes');
    img = getappdata(fig, 'Image');

    if isempty(img)
        uialert(fig, 'Please load an image first.', 'Error');
        return;
    end

    % Remove previous vanishing point if it exists
    removeExistingVanishingPoint(fig);
    
    % Draw a new point for the vanishing point
    vpHandle = drawpoint(ax, 'Color', 'r');
    pos = vpHandle.Position;

    % Set initial vanishing point in app data
    setappdata(fig, 'VanishingPoint', pos);
    setappdata(fig, 'VanishingPointHandle', vpHandle);

    % Draw initial mesh lines
    drawMeshLines(fig, ax, pos);

    % Attach a listener to the point to handle dragging
    addlistener(vpHandle, 'MovingROI', @(src, evt) vanishingPointMoved(fig, ax, src.Position));
end

function vanishingPointMoved(fig, ax, pos)
    setappdata(fig, 'VanishingPoint', pos);
    drawMeshLines(fig, ax, pos);
end

function removeExistingVanishingPoint(fig)
    oldVpHandle = getappdata(fig, 'VanishingPointHandle');
    if ~isempty(oldVpHandle) && isvalid(oldVpHandle)
        delete(oldVpHandle);
    end

    oldMeshLines = getappdata(fig, 'MeshLines');
    if ~isempty(oldMeshLines)
        delete(oldMeshLines);
    end
end

% 3.2 Determine the inner Rectangle
function drawInnerRectangle(fig)
    ax = getappdata(fig, 'BackgroundAxes');
    img = getappdata(fig, 'Image');

    if isempty(img)
        uialert(fig, 'Please load an image first.', 'Error');
        return;
    end
    
    % Remove existing rectangle if present
    removeExistingRectangle(fig);

    % Draw a new rectangle
    rect = drawrectangle(ax, 'Color', 'g', 'LineWidth', 2);
    addlistener(rect, 'MovingROI', @(src, evt) rectangleMoved(fig, ax, rect));
    addlistener(rect, 'ROIMoved', @(src, evt) rectangleMoved(fig, ax, rect));

    setappdata(fig, 'RectangleHandle', rect);
end

function rectangleMoved(fig, ax, rect)
    setappdata(fig, 'InnerRectangle', rect.Position);
    drawMeshLines(fig, ax, getappdata(fig, 'VanishingPoint'));
end

function removeExistingRectangle(fig)
    oldRectHandle = getappdata(fig, 'RectangleHandle');
    if ~isempty(oldRectHandle) && isvalid(oldRectHandle)
        delete(oldRectHandle);
    end
end

% 3.3 Draw prespective Lines
function drawMeshLines(fig, ax, vanishingPoint)
    % Clear old mesh lines
    oldMeshLines = getappdata(fig, 'MeshLines');
    if ~isempty(oldMeshLines)
        delete(oldMeshLines);
    end

    % Get inner rectangle
    innerRectangle = getappdata(fig, 'InnerRectangle');
    if isempty(innerRectangle)
        return;
    end
    
    % Compute corners of the inner rectangle
    rect_x = innerRectangle(1);
    rect_y = innerRectangle(2);
    rect_w = innerRectangle(3);
    rect_h = innerRectangle(4);
    corners = [
        rect_x, rect_y;
        rect_x + rect_w, rect_y;
        rect_x + rect_w, rect_y + rect_h;
        rect_x, rect_y + rect_h
    ];

    img = getappdata(fig, 'Image');
    imgHeight = size(img, 1);
    imgWidth = size(img, 2);

    % Draw the prespective lines
    hold(ax, 'on');
    meshLines = gobjects(4, 1);
    for i = 1:size(corners, 1)
        [xInter, yInter] = getBoundaryIntersections(vanishingPoint(1), vanishingPoint(2), corners(i, 1), corners(i, 2), imgWidth, imgHeight);
        meshLines(i) = plot(ax, [vanishingPoint(1), xInter], [vanishingPoint(2), yInter], 'w-','LineWidth', 0.5);
    end
    hold(ax, 'off');
    
    setappdata(fig, 'MeshLines', meshLines);
end

function [x_inter, y_inter] = getBoundaryIntersections(vp_x, vp_y, corner_x, corner_y, imgWidth, imgHeight)
    % Calculate the slope of the line from the vanishing point to the corner
    slope = (corner_y - vp_y) / (corner_x - vp_x);

    % Calculate the intersection points with the image boundaries
    if corner_x > vp_x
        % Right boundary
        x_inter = imgWidth;
        y_inter = vp_y + slope * (imgWidth - vp_x);
    else
        % Left boundary
        x_inter = 1;
        y_inter = vp_y + slope * (1 - vp_x);
    end

    if y_inter > imgHeight
        % Bottom boundary
        y_inter = imgHeight;
        x_inter = vp_x + (imgHeight - vp_y) / slope;
    elseif y_inter < 1
        % Top boundary
        y_inter = 1;
        x_inter = vp_x + (1 - vp_y) / slope;
    end
end

%% 4.Compute prespective Transformation
function startAnimation(fig)
    img = getappdata(fig, 'Image');
    vanishingPoint = getappdata(fig, 'VanishingPoint');
    innerRectangle = getappdata(fig, 'InnerRectangle');

    if isempty(img) || isempty(vanishingPoint) || isempty(innerRectangle)
        uialert(fig, 'Please load image, select vanishing point, and draw inner rectangle.', 'Error');
        return;
    end

    % Coordinates of the inner rectangle and vanishing point
    rect_x = innerRectangle(1);
    rect_y = innerRectangle(2);
    rect_w = innerRectangle(3);
    rect_h = innerRectangle(4);
    vanishing_x = vanishingPoint(1);
    vanishing_y = vanishingPoint(2);

    % Calculate vectors from vanishing point to the corners of the inner rectangle
    rectangle_x = [rect_x; rect_x+rect_w; rect_x+rect_w; rect_x];
    rectangle_y = [rect_y; rect_y; rect_y+rect_h; rect_y+rect_h];

    prespectiveTransform(fig, img, rectangle_x, rectangle_y, vanishing_x, vanishing_y);
end

% 4.1 Calculate the prespective transformed 3D-Points to construct 3D-Scene
function prespectiveTransform(fig, img, rectangle_x, rectangle_y, vanishing_point_x, vanishing_point_y)
    % Determine the size of the image to set boundaries for intersections
    end_x = size(img, 2);
    end_y = size(img, 1);
    
    % Calculate vectors from the vanishing point to each corner of the rectangle
    vector_top_left = [rectangle_x(1)-vanishing_point_x; rectangle_y(1)-vanishing_point_y];
    vector_top_right = [rectangle_x(2)-vanishing_point_x; rectangle_y(2)-vanishing_point_y];
    vector_bottom_right = [rectangle_x(3)-vanishing_point_x; rectangle_y(3)-vanishing_point_y];
    vector_bottom_left = [rectangle_x(4)-vanishing_point_x; rectangle_y(4)-vanishing_point_y];
    
    vanishing_point = [vanishing_point_x; vanishing_point_y];
    
    % Calculate intersections of these vectors with the image borders
    intersection_tl_left = intersection_with_image_border(vanishing_point, vector_top_left, [0; 0], [0; 1]);
    intersection_tl_top = intersection_with_image_border(vanishing_point, vector_top_left, [0; 0], [1; 0]);
    
    intersection_tr_right = intersection_with_image_border(vanishing_point, vector_top_right, [end_x; 0], [0; 1]);
    intersection_tr_top = intersection_with_image_border(vanishing_point, vector_top_right, [end_x; 0], [1; 0]);
    
    intersection_br_right = intersection_with_image_border(vanishing_point, vector_bottom_right, [end_x; end_y], [0; 1]);
    intersection_br_bottom = intersection_with_image_border(vanishing_point, vector_bottom_right, [end_x; end_y], [1; 0]);
    
    intersection_bl_left = intersection_with_image_border(vanishing_point, vector_bottom_left, [0; end_y], [0; 1]);
    intersection_bl_bottom = intersection_with_image_border(vanishing_point, vector_bottom_left, [0; end_y], [1; 0]);
    
    intersections = [intersection_tl_left, intersection_tl_top, intersection_tr_right, intersection_tr_top, intersection_br_right, intersection_br_bottom, intersection_bl_left, intersection_bl_bottom];
    
    % Calculate the height based on the minimal distance between top and bottom intersections
    height = minDistance([rectangle_x(1); rectangle_y(1)], [rectangle_x(4); rectangle_y(4)], [rectangle_x(2); rectangle_y(2)], [rectangle_x(3); rectangle_y(3)]);
    
    % Use the intersections and height to create 3D points that represent the rectangle in 3D space
    threed_rectangle_bottom_left = [rectangle_x(4); 0; rectangle_y(4)];
    threed_rectangle_bottom_right = [rectangle_x(3); 0; rectangle_y(3)];
    threed_rectangle_top_left = [threed_rectangle_bottom_left(1); height; threed_rectangle_bottom_left(3)];
    threed_rectangle_top_right = [threed_rectangle_bottom_right(1); height; threed_rectangle_bottom_right(3)];
    
    % Calculate distances for additional 3D positioning based on intersections and rectangle geometry
    distance_bottom = minDistance([intersection_br_bottom(1); intersection_br_bottom(2)], [rectangle_x(3); rectangle_y(3)], [intersection_bl_bottom(1); intersection_bl_bottom(2)], [rectangle_x(4); rectangle_y(4)]);
    distance_top = minDistance([intersection_tr_top(1); intersection_tr_top(2)], [rectangle_x(2); rectangle_y(2)], [intersection_tl_top(1); intersection_tl_top(2)], [rectangle_x(1); rectangle_y(1)]);
    distance_left = minDistance([intersection_tl_left(1); intersection_tl_left(2)], [rectangle_x(1); rectangle_y(1)],[intersection_bl_left(1); intersection_bl_left(2)], [rectangle_x(4); rectangle_y(4)]);
    distance_right = minDistance([intersection_tr_right(1); intersection_tr_right(2)], [rectangle_x(2); rectangle_y(2)],[intersection_br_right(1); intersection_br_right(2)], [rectangle_x(3); rectangle_y(3)]);
    
    % Calculate 90-degree points to help form the full 3D perspective of the rectangle
    threed_intersection_tl_left = ninetyDegreePoint(threed_rectangle_top_left, threed_rectangle_top_right-threed_rectangle_top_left, distance_left);
    threed_intersection_bl_left = ninetyDegreePoint(threed_rectangle_bottom_left, threed_rectangle_top_right-threed_rectangle_top_left, distance_left);
    
    threed_intersection_tr_right = ninetyDegreePoint(threed_rectangle_top_right, threed_rectangle_top_right-threed_rectangle_top_left, distance_right);
    threed_intersection_br_right = ninetyDegreePoint(threed_rectangle_bottom_right, threed_rectangle_top_right-threed_rectangle_top_left, distance_right);
    
    threed_intersection_tl_top = ninetyDegreePoint(threed_rectangle_top_left, threed_rectangle_top_right-threed_rectangle_top_left, distance_top);
    threed_intersection_tr_top = ninetyDegreePoint(threed_rectangle_top_right, threed_rectangle_top_right-threed_rectangle_top_left, distance_top);
    
    threed_intersection_bl_bottom = ninetyDegreePoint(threed_rectangle_bottom_left, threed_rectangle_top_right-threed_rectangle_top_left, distance_bottom);
    threed_intersection_br_bottom = ninetyDegreePoint(threed_rectangle_bottom_right, threed_rectangle_top_right-threed_rectangle_top_left, distance_bottom);
    
    % Combine all 3D points into a single array for visualization
    threed_points = [threed_rectangle_bottom_right, threed_rectangle_bottom_left, threed_rectangle_top_right, threed_rectangle_top_left, threed_intersection_br_right, threed_intersection_br_bottom, threed_intersection_bl_left, threed_intersection_bl_bottom, threed_intersection_tl_left, threed_intersection_tl_top, threed_intersection_tr_right, threed_intersection_tr_top];
    
    % Display the transformed segments in the GUI, representing the 3D perspective
    displayTransformedSegments(fig, img, threed_points, rectangle_x, rectangle_y, intersections);
end

% Calculate intersection of the vanishing point vectors with the image border
function intersection_with_border = intersection_with_image_border(vanishing_point, direction, border_point, border_direction)
    % Construct the system of equations
    A = [direction(:), -border_direction(:)];
    b = border_point(:) - vanishing_point(:);
    
    % Solve the system of linear equations
    t = A \ b;
    
    % Calculate the intersection point
    intersection_with_border = vanishing_point + t(1) * direction;
end

% Calculate the Euclidean distance between two points projected onto vectors
function [min_distance] = minDistance(v1, r1, v2, r2)
    % v1, r1: The first point and its projection vector
    % v2, r2: The second point and its projection vector
    min_distance = min(norm(v1-r1), norm(v2-r2));
end

% Calculate a new point at a distance from a starting point in a direction 
% rotated 90 degrees from the original direction
function [point] = ninetyDegreePoint(startingPoint, originalDirection, distance)
    % Normalize the original direction vector to get a unit vector
    unitOriginalDirection = originalDirection / norm(originalDirection);

    % Rotate the direction vector by 90 degrees in the plane; here assumed a rotation
    % in 3D by swapping and negating coordinates to achieve orthogonality
    newDirection = [-unitOriginalDirection(3); unitOriginalDirection(2); unitOriginalDirection(1)] * distance;
    
    % Calculate the new point by adding the rotated direction vector to the starting point
    point = startingPoint + newDirection;
end

%% 5.Display the 3D-Scene in the GUI
function displayTransformedSegments(fig, img, threed_points, rectangle_x, rectangle_y, intersections)  
    % Retrieve center points of the rectangle
    center_points = [rectangle_x rectangle_y];  % Defines the center of the projection area
    
    % Define the polygon vertices for each segment
    segments = {
        [intersections(:,8)'; center_points(4,:); center_points(3,:); intersections(:,6)']; % bottom face
        [center_points(2,:); intersections(:,3)'; intersections(:,5)'; center_points(3,:)]; % right face
        [intersections(:,1)'; center_points(1,:); center_points(4,:); intersections(:,7)']; % left face
        center_points; % back face
        [intersections(:,2)'; intersections(:,4)'; center_points(2,:); center_points(1,:)]; % top face
    };

    % % Define the output width and height for each transformed segment
    output_width_and_height = {
        [norm(threed_points(1,2)-threed_points(1,1)); norm(threed_points(3,6)-threed_points(3,1))]; % bottom
        [norm(threed_points(3,5)-threed_points(3,1)); norm(threed_points(2,1)-threed_points(2,3))]; % right
        [norm(threed_points(3,7)-threed_points(3,2)); norm(threed_points(2,1)-threed_points(2,3))]; % left
        [norm(threed_points(1,2)-threed_points(1,1)); norm(threed_points(2,1)-threed_points(2,3))]; % back
        [norm(threed_points(1,2)-threed_points(1,1)); norm(threed_points(3,3)-threed_points(3,11))]; % top
    };

    % Initialize an array to store each transformed segment image
    transformed_segment_images = cell(length(segments), 1);
    %transformed_foreground_images = cell(length(segments), 1);

    % Process each segment for background and foreground transformations
    for i = 1:length(segments)
        % Background
        % Define the source points (corners of the trapezoid)
        source_points = segments{i};
        
        % Define the destination points (corners of the rectangle)
        output_width_and_height_for_item = [round(output_width_and_height{i}(1)); round(output_width_and_height{i}(2))];
        dest_points = [
            0, 0;
            output_width_and_height_for_item(1), 0;
            output_width_and_height_for_item(1), output_width_and_height_for_item(2);
            0, output_width_and_height_for_item(2)
        ];

        % Calculate the transformation of Background
        tform = fitgeotrans(source_points, dest_points, 'projective');
        
        % Apply the transformation to the Background
        transformed_img = imwarp(img, tform, 'OutputView', imref2d([output_width_and_height_for_item(2) output_width_and_height_for_item(1)]));
        
        % Store the transformed image
        transformed_segment_images{i} = transformed_img;
    end
    
    transformed_segment_images{1} = flip(transformed_segment_images{1}, 2);
    transformed_segment_images{3} = flip(transformed_segment_images{3}, 2);
    transformed_segment_images{4} = imrotate(transformed_segment_images{4}, 270);
    transformed_segment_images{5} = imrotate(transformed_segment_images{5}, 270);
    transformed_segment_images{5} = flip(transformed_segment_images{5}, 1);
    % Drehe das Bild fuer das 5. Polygon um 270 Grad
    
    % Zeige die transformierten Segmente auf den Polygonen an
    vertices = threed_points.';

    % Tausche die x- und z-Koordinaten
    vertices = vertices(:, [1, 3, 2]);

    % Definiere die Fluechen (jede Zelle beschreibt eine Flueche unter Verwendung der Indizes der Eckpunkte)
    faces = {
        [2 1 8 6];       % Boden
        [3 1 11 5];      % Rechte Wand
        [4 2 9 7];       % Linke Wand
        [2 1 4 3];       % Rueckwand
        [3 4 12 10];     % Decke
    };

    % Definiere die Zuordnung der transformierten Bilder zu den Fluechen
    image_to_face_mapping = [1, 2, 3, 4, 5];

    % Set axes properties for 3D display
    ax = getappdata(fig, 'BackgroundAxes');
    cla(ax);
    hold(ax, 'on');

    % Draw the transformed segments on the respective polygon surfaces
    for i = 1:length(segments)
        % Retrieve vertices for the current face
        face_vertices = vertices(faces{i}, :);

        % Convert face vertices to form a surface
        x = reshape(face_vertices(:, 1), [2, 2]);
        y = reshape(face_vertices(:, 2), [2, 2]);
        z = reshape(face_vertices(:, 3), [2, 2]);

        % Retrieve the associated image for the current face
        img_index = image_to_face_mapping(i);
        img_to_use = transformed_segment_images{img_index};

        % Create the surface with the texture of the transformed segment image
        surface(ax, x, y, z, 'FaceColor', 'texturemap', 'CData', img_to_use, 'EdgeColor', 'none'); 
    end

    hold(ax, 'off');
    rotate3d(ax, 'on');
    axis(ax, 'vis3d');
    view(ax, 3);
end

function projectForeground(fig)
    foregroundImage = getappdata(fig, 'ForegroundImage');
    innerRectangle = getappdata(fig, 'InnerRectangle');
    vanishingPoint = getappdata(fig, 'VanishingPoint');
    foregroundPosition = getappdata(fig, 'ForegroundPositions');

    disp('Foreground Position:');
    disp(foregroundPosition);

    % Define the projection plane
    plane = defineProjectionPlane(innerRectangle, vanishingPoint, foregroundPosition);
    disp('Plane Corners:');
    disp(plane.corners);

    % Draw the projection plane, projection of the foreground is not yet done
    drawProjectionPlane(fig, plane);

    % Project the foreground onto the plane (this line is commented out for now)
    % projectForegroundToPlane(fig, foregroundImage, plane);
end

function plane = defineProjectionPlane(innerRectangle, vanishingPoint, foregroundPosition)
    % Calculate the vertical depth relative to the inner rectangle

    % Assume the plane needs to be at the back wall, depth on the z-axis is 0 (adjusted for swapped axes)
    plane.corners = [
        innerRectangle(1), 0, innerRectangle(2);  % Use z = innerRectangle(2) to adapt to the swapped coordinate system
        innerRectangle(1) + innerRectangle(3), 0, innerRectangle(2);
        innerRectangle(1) + innerRectangle(3), 0, innerRectangle(2) + innerRectangle(4);
        innerRectangle(1), 0, innerRectangle(2) + innerRectangle(4)
    ];
    
    % Use the vanishing point and foreground position to determine projection depth
    % depth = calculateDepth(vanishingPoint, innerRectangle, foregroundPosition);
    depth = 0;  % Placeholder for depth calculation
    disp('Foreground depth:');
    disp(depth);
end

function drawProjectionPlane(fig, plane)
    ax = getappdata(fig, 'BackgroundAxes');
    corners = plane.corners;

    % Draw an unfilled polygon to represent the projection plane
    hold(ax, 'on');
    % Ensure the plane is closed
    cornersExtended = [corners; corners(1,:)];  % Add the first point to the end to ensure closure
    % Draw the polygon, use plot3 to adapt to the 3D scene
    plot3(ax, cornersExtended(:,3), cornersExtended(:,2), -cornersExtended(:,1), 'r-', 'LineWidth', 2);  % Adjust y and z to fit the 3D scene's coordinate system
    hold(ax, 'off');
    drawnow;  % Ensure the drawing updates immediately
end

function projectForegroundToPlane(fig, foregroundImage, plane)
    ax = getappdata(fig, 'BackgroundAxes');

    % Retrieve the corner coordinates of the plane
    corners = plane.corners;

    % Define the source points of the corners of the foreground image
    sourcePoints = [1, 1; size(foregroundImage, 2), 1; size(foregroundImage, 2), size(foregroundImage, 1); 1, size(foregroundImage, 1)];
    destPoints = corners(:, 1:2);  % Only use x and y as destination points

    % Create the transformation matrix
    tform = fitgeotrans(sourcePoints, destPoints, 'projective');
    
    % Calculate the dimensions of the transformed output image, ensuring integer values
    outputHeight = round(max(destPoints(:,2)) - min(destPoints(:,2)));
    outputWidth = round(max(destPoints(:,1)) - min(destPoints(:,1)));

    % Create a reference object for the output view
    outputView = imref2d([outputHeight, outputWidth]);

    % Apply the transformation
    warpedImage = imwarp(foregroundImage, tform, 'OutputView', outputView);
    
    % Maintain the current content of Axes
    hold(ax, 'on');

    % Draw the transformed foreground image
    h = imagesc(ax, [min(destPoints(:,1)), max(destPoints(:,1))], [min(destPoints(:,2)), max(destPoints(:,2))], warpedImage);

    % Set transparency, adjust as necessary
    set(h, 'AlphaData', ~all(warpedImage == 0, 3));

    % Preserve the current view
    hold(ax, 'off');
end

function depth = calculateDepth(vanishingPoint, innerRectangle, foregroundPosition)
    % Determine the projection depth based on the vanishing point and foreground position
    % This is a simplified calculation; adjust the logic for depth calculation as necessary
    
    % For example, depth can be estimated based on the vertical distance from the vanishing point to the foreground position
    % Simplified here as the vertical distance from the vanishing point to the center of the inner rectangle plus an adjustment factor based on the foreground position
    rectCenterY = innerRectangle(2) + innerRectangle(4) / 2;
    depthFactor = abs(vanishingPoint(2) - rectCenterY);  % Vertical distance from the vanishing point to the center of the rectangle
    foregroundAdjustment = abs(foregroundPosition(2) - rectCenterY);  % Adjustment based on the foreground position

    % Calculate the final depth
    depth = depthFactor + foregroundAdjustment / 2;  % Simple weighted adjustment
end