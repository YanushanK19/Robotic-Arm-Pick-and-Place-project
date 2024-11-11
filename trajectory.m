clear;
clc;

% Define the robot links
L(1) = Link([0,     290,    0,       -pi/2]);
L(2) = Link([-pi/2, 0 ,     270,     0    ]);
L(3) = Link([0,     0 ,     70,      -pi/2]);
L(4) = Link([0,     302,    0 ,      pi/2]);
L(5) = Link([0,     0 ,     0 ,      -pi/2]);
L(6) = Link([0,     72,     0 ,      0  ]);

% Create the robot model
IRB_120 = SerialLink(L);

% Define the target points (in mm)
points = [
    374  0    630;
    0    450  580;
    0    450  285;
    0    450  580;
    400  0    580;
    400  0    461;
    400  0    580;
    100  0    580;
    400  0    580;
    400  0    461;
    400  0    580;
    0    -450 580;
    0    -450 281;
    0    -450 580;
    374  0    630
];

% Number of samples between points
samples = 10;

% Initialize arrays for trajectory points
x = [];
y = [];
z = [];

% Generate trajectory points
for i = 1:size(points, 1)-1
    % Interpolate points between the current and next target
    x_temp = linspace(points(i, 1), points(i+1, 1), samples);
    y_temp = linspace(points(i, 2), points(i+1, 2), samples);
    z_temp = linspace(points(i, 3), points(i+1, 3), samples);
    
    % Append points
    x = [x; x_temp'];
    y = [y; y_temp'];
    z = [z; z_temp'];
end

% Initialize the transformation matrices
T = zeros(4, 4, length(x));

% Create transformation matrices for each point
for i = 1:length(x)
    T(:,:,i) = [1, 0, 0, x(i);
                0, 1, 0, y(i);
                0, 0, 1, z(i);
                0, 0, 0, 1];
end

% Perform inverse kinematics for each pose
q = zeros(length(x), 6);
for i = 1:length(x)
    q_sol = IRB_120.ikine(T(:,:,i), 'mask', [1 1 1 0 0 0]);
    if ~any(isnan(q_sol))
        q(i, :) = q_sol;
    else
        fprintf('No valid solution found for pose %d.\n', i);
    end
end

% Plot the robot motion and the trajectory
figure;
IRB_120.plot(q, 'trail', '-'); % Plot the robot motion with a trail
hold on;
plot3(x, y, z, 'r--o'); % Plot the trajectory points
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Robot Trajectory Planning');
grid on;

