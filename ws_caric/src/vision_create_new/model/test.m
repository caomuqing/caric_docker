% Define the line
line_start = [-25, 0, 0];
line_end = [25, 0, 0];

% Generate points along the line
num_points = 100;
t = linspace(0, 1, num_points);
line_points = (1 - t)' * line_start + t' * line_end;

% Define the transformation matrix from the provided orientation
orientation_matrix = [
    0.905489325523, -0.424368888140, 0.000000000000, 14.338454246521;
    0.424368888140, 0.905489325523, 0.000000000000, -15.624868392944;
    0.000000000000, 0.000000000000, 1.000000000000, 7.3;
    0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000
];

% Apply the transformation to the line points
transformed_line_points = (orientation_matrix(1:3, 1:3) * line_points' + orientation_matrix(1:3, 4))';

% Define the second line
line2_start = [-20.5, 0, 0];
line2_end = [20.5, 0, 0];

% Generate points along the second line
line2_points = (1 - t)' * line2_start + t' * line2_end;

% Define the transformation matrix for the second line
orientation_matrix_line2 = [
    0.877950966358, 0.478735953569, -0.003650249913, -19.381038665771;
   -0.478749573231, 0.877926290035, -0.006693980191, -14.002973556519;
    0.000000000000, 0.007624549791,  0.999970853329,  7.287963867188;
    0.000000000000, 0.000000000000,  0.000000000000,  1.000000000000
];

% Apply the transformation to the second line points
transformed_line2_points = (orientation_matrix_line2(1:3, 1:3) * line2_points' + orientation_matrix_line2(1:3, 4))';


% Plot the original and transformed points
figure;
plot3(line_points(:,1), line_points(:,2), line_points(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(transformed_line_points(:,1), transformed_line_points(:,2), transformed_line_points(:,3), 'r-', 'LineWidth', 2);
scatter3(14.33845425, -15.62486839, 7.3, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g');
% Plot the second line
plot3(transformed_line2_points(:,1), transformed_line2_points(:,2), transformed_line2_points(:,3), 'g-', 'LineWidth', 2);
scatter3(-19.38103867, -14.00297356, 7.28796387, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'b');
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Original Line', 'Transformed Line', 'Box Center', 'Location', 'Best');
title('Line Transformation');

