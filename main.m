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

%% load Image
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

%% get the foreground objects
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
end

%% get the vanishing point
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

%% get the inner Rectangle
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

%% draw prespective Lines
function drawMeshLines(fig, ax, vanishingPoint)
    % clear old mesh lines
    oldMeshLines = getappdata(fig, 'MeshLines');
    if ~isempty(oldMeshLines)
        delete(oldMeshLines);
    end

    % get inner rectangle
    innerRectangle = getappdata(fig, 'InnerRectangle');
    if isempty(innerRectangle)
        return;
    end
    
    % compute corners of the inner rectangle
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

    % draw the prespective lines
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

%% Tour into the Picture
function startAnimation(fig)
    img = getappdata(fig, 'Image');
    vanishingPoint = getappdata(fig, 'VanishingPoint');
    innerRectangle = getappdata(fig, 'InnerRectangle');

    if isempty(img) || isempty(vanishingPoint) || isempty(innerRectangle)
        uialert(fig, 'Please load image, select vanishing point, and draw inner rectangle.', 'Error');
        return;
    end

    % Create spidery mesh based on inner rectangle and vanishing point
    rect_x = innerRectangle(1);
    rect_y = innerRectangle(2);
    rect_w = innerRectangle(3);
    rect_h = innerRectangle(4);
    vanishing_x = vanishingPoint(1);
    vanishing_y = vanishingPoint(2);

    rectangle_x = [rect_x; rect_x+rect_w; rect_x+rect_w; rect_x];
    rectangle_y = [rect_y; rect_y; rect_y+rect_h; rect_y+rect_h];

    animateTIP(fig, img, rectangle_x, rectangle_y, vanishing_x, vanishing_y);
end


function animateTIP(fig, img, rectangle_x, rectangle_y, vanishing_point_x, vanishing_point_y)
    
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


function displayTransformedSegments(fig, img, threed_points, rectangle_x, rectangle_y, intersections)
    % Trenne die Mittelpunkte und Randpunkte aus der Intersections-Matrix
    center_points = [rectangle_x rectangle_y];
    
    % Definiere die Polygon-Eckpunkte fuer jedes Segment
    segments = {
        [intersections(:,8)'; center_points(4,:); center_points(3,:); intersections(:,6)']; % bottom
        [center_points(2,:); intersections(:,3)'; intersections(:,5)'; center_points(3,:)]; % right
        [intersections(:,1)'; center_points(1,:); center_points(4,:); intersections(:,7)']; % left
        center_points; % back
        [intersections(:,2)'; intersections(:,4)'; center_points(2,:); center_points(1,:)]; % top
    };

    output_width_and_height = {
        [norm(threed_points(1,2)-threed_points(1,1)); norm(threed_points(3,6)-threed_points(3,1))]; % bottom
        [norm(threed_points(3,5)-threed_points(3,1)); norm(threed_points(2,1)-threed_points(2,3))]; % right
        [norm(threed_points(3,7)-threed_points(3,2)); norm(threed_points(2,1)-threed_points(2,3))]; % left
        [norm(threed_points(1,2)-threed_points(1,1)); norm(threed_points(2,1)-threed_points(2,3))]; % back
        [norm(threed_points(1,2)-threed_points(1,1)); norm(threed_points(3,3)-threed_points(3,11))]; % top
        
    };

    % Initialisiere Zellarray zur Speicherung jedes transformierten Segmentbildes
    transformed_segment_images = cell(length(segments), 1);

    % Grueuee des rechteckigen Ausgabebildes 

    face_mapping = {
        [4 3 1 2];       % Boden
        [1 3 4 2];      % Rechte Wand
        [3 1 2 4];       % Linke Wand
        [3 4 2 1];       % Rueckwand
        [4 3 1 2];     % Decke
    };

    % Schneide, transformiere und speichere jedes Segment
    for i = 1:length(segments)
        % Definiere die Ausgangspunkte (Eckpunkte des Trapezes)
        source_points = segments{i};
        
        % Definiere die Zielpunkte (Eckpunkte des Rechtecks)
        output_width_and_height_for_item = [round(output_width_and_height{i}(1)); round(output_width_and_height{i}(2))]
        dest_points = [
            0, 0;
            output_width_and_height_for_item(1), 0;
            output_width_and_height_for_item(1), output_width_and_height_for_item(2);
            0, output_width_and_height_for_item(2)
        ];

        new_dest_points = zeros(size(dest_points))
        for j = 1:4
            new_dest_points(face_mapping{i}(j), :) = dest_points(j,:);
        end

        % Berechne die Transformationsmatrix
        tform = fitgeotrans(source_points, dest_points, 'projective');
        
        % Wende die Transformation an
        transformed_img = imwarp(img, tform, 'OutputView', imref2d([output_width_and_height_for_item(2) output_width_and_height_for_item(1)]));
        
        % Spiegele das transformierte Bild horizontal
        %if i==2 || i==4
        %    transformed_img = flip(transformed_img, 2);
        %end
        
        % Speichere das gespiegelte Bild
        transformed_segment_images{i} = transformed_img;
    end

    
    transformed_segment_images{1} = flip(transformed_segment_images{1}, 2)
    transformed_segment_images{3} = flip(transformed_segment_images{3}, 2)
    transformed_segment_images{4} = imrotate(transformed_segment_images{4}, 270);
    transformed_segment_images{5} = imrotate(transformed_segment_images{5}, 270);
    transformed_segment_images{5} = flip(transformed_segment_images{5}, 1);
    % Drehe das Bild fuer das 5. Polygon um 270 Grad
    
    % Spiegele das vierte Bild horizontal
    %transformed_segment_images{4} = flip(transformed_segment_images{4}, 1);
    
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
    %{
    faces = {
        [2 1 6 8];       % Boden
        [3 11 5 1];      % Rechte Wand
        [9 4 2 7];       % Linke Wand
        [4 3 1 2];       % Rueckwand
        [10 12 3 4];     % Decke
    };
    %}

    % Definiere die Zuordnung der transformierten Bilder zu den Fluechen
    image_to_face_mapping = [1, 2, 3, 4, 5]; 

    % Erstelle das Patch-Objekt mit Texturabbildung
    ax = getappdata(fig, 'BackgroundAxes');

    % Clear the axes and plot the initial view
    cla(ax);
    
    hold(ax, 'on');
    ax.Clipping = 'off';
    ax.SortMethod = 'childorder';

    for i = 1:length(faces)
        % Hole die Eckpunkte fuer die aktuelle Flueche
        face_vertices = vertices(faces{i}, :);
        
        % Erstelle eine Oberflueche fuer die aktuelle Flueche
        x = face_vertices(:, 1);
        y = face_vertices(:, 2);
        z = face_vertices(:, 3);
        
        % Forme die Eckpunkte um, um eine Oberflueche zu bilden
        x = reshape(x, [2, 2]);
        y = reshape(y, [2, 2]);
        z = reshape(z, [2, 2]);
        
        % Hole das zugeordnete Bild fuer die aktuelle Flueche
        img_index = image_to_face_mapping(i);
        img_to_use = transformed_segment_images{img_index};
        
        % Erstelle die Oberflueche mit der Textur des transformierten Segmentbildes
        surface(ax, x, y, z, 'FaceColor', 'texturemap', 'CData', img_to_use, 'EdgeColor', 'none');
    end
    
    %campos(ax, camera_position);
    %camva(ax, 90);
    hold(ax, 'off');

    % Enable 3D rotation for navigation
    view(ax, 0, 0);
    axis(ax, 'vis3d');
    zoom(ax, "on");
    rotate3d(ax, 'on');
end
