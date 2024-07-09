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

%% Foreground projection
function projectForeground(fig)
    ax = getappdata(fig, 'BackgroundAxes');
    img = getappdata(fig, 'Image');
    foreground = getappdata(fig, 'ForegroundImage');
    vanishingPoint = getappdata(fig, 'VanishingPoint');
    innerRectangle = getappdata(fig, 'InnerRectangle');

    if isempty(img) || isempty(foreground) || isempty(vanishingPoint) || isempty(innerRectangle)
        uialert(fig, 'Please ensure an image, foreground, vanishing point, and inner rectangle are all set.', 'Error');
        return;
    end

    % compute prespective Transform
    tform = computePerspectiveTransform(vanishingPoint, innerRectangle, size(img));
    transformedForeground = imwarp(foreground, tform, 'OutputView', imref2d(size(img)));

    % generate a mask to combine the foreground and backfround
    mask = transformedForeground(:, :, 1) > 0 | transformedForeground(:, :, 2) > 0 | transformedForeground(:, :, 3) > 0;
    img(mask) = transformedForeground(mask);

    % display combined Image
    imshow(img, 'Parent', ax);
    setappdata(fig, 'Image', img);
end