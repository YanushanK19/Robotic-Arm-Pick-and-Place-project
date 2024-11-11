% Define the 6-DOF robotic arm using DH parameters
L1 = Link('d', 0.4, 'a', 0.025, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 0.455, 'alpha', 0, 'offset', pi/2);
L3 = Link('d', 0, 'a', 0.035, 'alpha', pi/2);
L4 = Link('d', 0.42, 'a', 0, 'alpha', -pi/2);
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L6 = Link('d', 0.08, 'a', 0, 'alpha', 0);
 
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6DOF Arm');
 
% Define the positions of the boxes (left, center, right)
left_box_position = [0, 0.45, 0.15];   % Position of the left box
center_box_position = [0.75, 0.0, 0.3]; % Position of the center box
right_box_position = [0, -0.45, 0.1]; % Position of the right box
 
% Step 1: Pick the cube from the left box
target_position_left = transl(left_box_position(1), left_box_position(2), left_box_position(3));
q_pick_left = robot.ikine(target_position_left, 'mask', [1 1 1 0 0 0]); % Inverse kinematics for left box
 
% Step 2: Move the cube to the center box
target_position_center = transl(center_box_position(1), center_box_position(2), center_box_position(3));
q_place_center = robot.ikine(target_position_center, 'mask', [1 1 1 0 0 0]); % Inverse kinematics for center box
 
% Step 3: Pause to simulate delay
pause_time = 2; % Delay in seconds
 
% Step 4: Pick the cube from the center box
q_pick_center = q_place_center; % Use the same position as where it was placed
 
% Step 5: Move the cube to the right box
target_position_right = transl(right_box_position(1), right_box_position(2), right_box_position(3));
q_place_right = robot.ikine(target_position_right, 'mask', [1 1 1 0 0 0]); % Inverse kinematics for right box
 
% Visualize the robot picking and placing the cube
figure;
robot.plot(q_pick_left);
hold on;
plot3(left_box_position(1), left_box_position(2), left_box_position(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot3(center_box_position(1), center_box_position(2), center_box_position(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(right_box_position(1), right_box_position(2), right_box_position(3), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
 
% Move from left box to center box
trajectory_to_center = jtraj(q_pick_left, q_place_center, 50); % Generate trajectory
robot.plot(trajectory_to_center); % Plot the trajectory
 
% Pause for delay
pause(pause_time);
 
% Move from center box to right box
trajectory_to_right = jtraj(q_pick_center, q_place_right, 50); % Generate trajectory
robot.plot(trajectory_to_right); % Plot the trajectory
 
% Verification of Forward and Inverse Kinematics
disp('Verification of Forward and Inverse Kinematics:');

% Compute Inverse Kinematics for each target position
ik_pick_left_check = robot.ikine(target_position_left, 'mask', [1 1 1 0 0 0]);
disp('Inverse Kinematics Check (Pick Left):');
disp(ik_pick_left_check);

ik_place_center_check = robot.ikine(target_position_center, 'mask', [1 1 1 0 0 0]);
disp('Inverse Kinematics Check (Place Center):');
disp(ik_place_center_check);

ik_place_right_check = robot.ikine(target_position_right, 'mask', [1 1 1 0 0 0]);
disp('Inverse Kinematics Check (Place Right):');
disp(ik_place_right_check);

disp('Simulation completed.');
