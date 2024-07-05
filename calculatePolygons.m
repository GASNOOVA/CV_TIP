% Erstellen Sie die Vertices (Punkte) des 3D-Objekts
% vertices = [
%     0 0 0;  % Punkt 1
%     0 2 0;  % Punkt 2
%     2 0 0;  % Punkt 3
%     2 2 0;  % Punkt 4
%     3 0 0;  % Punkt 5
%     1 2 0;  % Punkt 6
%     0 0 2;  % Punkt 7
%     0 2 2;  % Punkt 8
%     2 0 2;  % Punkt 9
%     2 2 2;  % Punkt 10
%     3 0 2;  % Punkt 11
%     1 2 2;  % Punkt 12
% ];

% Laden Sie das Bild
img = imread('oil-painting.png');

vertices = threed_points.';

% Erstellen Sie die Flächen (jede Zelle beschreibt eine Fläche anhand der Indizes der Vertices)
faces = {
    [2 1 4 3]       % Rückwand
    [4 2 9 7];      % Linke Wand
    [3 1 12 5];     % Rechte Wand
    [3 4 11 10];     % Decke
    [2 1 8 6];    % Boden
};

% Erstellen Sie das Patch-Objekt mit Texturabbildung
figure;
hold on;
for i = 1:length(faces)
    % Get the vertices for the current face
    face_vertices = vertices(faces{i}, :);
    
    % Create a surface for the current face
    x = face_vertices(:, 1);
    y = face_vertices(:, 2);
    z = face_vertices(:, 3);
    
    % Reshape the vertices to form a surface
    x = reshape(x, [2, 2]);
    y = reshape(y, [2, 2]);
    z = reshape(z, [2, 2]);
    
    % Create the surface with the image texture
    surface(x, y, z, 'FaceColor', 'texturemap', 'CData', img, 'EdgeColor', 'none');
end
hold off;

% Achsenbeschriftungen und Ansicht anpassen
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;

% Maussteuerung aktivieren
rotate3d on;

% Plot aktualisieren
view(3);