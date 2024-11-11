q_left = [1.5721,   -0.9905,   -0.5722,    0.1082,    0.0685,        0];
q_center = [0.0000,   -0.9666,    0.0947,    0.0000,    0.0343 ,        0];
q_right = [-1.5731 ,  -1.1129,   -0.5105,   -0.1243,    0.1036,         0];

% Compute Forward Kinematics for each configuration
fk_pick_left = robot.fkine(q_left);
fk_place_center = robot.fkine(q_center);
fk_place_right = robot.fkine(q_right);

disp('Forward Kinematics Check (Pick Left):');
disp(fk_pick_left);

disp('Forward Kinematics Check (Place Center):');
disp(fk_place_center);

disp('Forward Kinematics Check (Place Right):');
disp(fk_place_right);

disp('Simulation completed.');