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