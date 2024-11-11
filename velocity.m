clear;
clc;

% Joint angles (in degrees)
q1 = [90.022907, 4.301263, 31.687908, 2.492668, 51.040679, 0.155065];
q2 = [90.000000, 35.497520, 35.221274, 0.000000, 19.281206, -0.000000];
q3 = [0.000000, -7.800398, 16.178010, 0.000000, 80.323297, -0.000000];
q4 = [0.000000, 9.865775, 40.223267, 0.000000, 38.611957, -0.000000];
q5 = [-89.977093, 4.301263, 31.687908, 2.492668, 51.040679, 0.155065];
q6 = [-90.000000, 36.766016, 35.299140, 0.000000, 17.705844, -0.000000];

% Convert to radians
q1 = deg2rad(q1);
q2 = deg2rad(q2);
q3 = deg2rad(q3);
q4 = deg2rad(q4);
q5 = deg2rad(q5);
q6 = deg2rad(q6);

% Time arrays for each trajectory segment
t1 = (0:0.005:0.905)';
t2 = (0:0.005:2.255)';
t3 = (0:0.005:4.055)';
t4 = (0:0.005:5.505)';
t5 = (0:0.005:10.455)';
t6 = (0:0.005:11.855)';

% Trajectory generation using linear interpolation
q_interp1 = repmat(q1, size(t1,1), 1) + (repmat(q2-q1, size(t1,1), 1) .* (t1 / max(t1)));
q_interp2 = repmat(q2, size(t2,1), 1) + (repmat(q3-q2, size(t2,1), 1) .* (t2 / max(t2)));
q_interp3 = repmat(q3, size(t3,1), 1) + (repmat(q4-q3, size(t3,1), 1) .* (t3 / max(t3)));
q_interp4 = repmat(q4, size(t4,1), 1) + (repmat(q5-q4, size(t4,1), 1) .* (t4 / max(t4)));
q_interp5 = repmat(q5, size(t5,1), 1) + (repmat(q6-q5, size(t5,1), 1) .* (t5 / max(t5)));
q_interp6 = repmat(q6, size(t6,1), 1) + (repmat(q1-q6, size(t6,1), 1) .* (t6 / max(t6)));

% Plotting joint angles
figure
subplot(2,1,1)
plot(t1, rad2deg(q_interp1(:,2)), 'DisplayName', 'Segment 1')
hold on
plot(t2, rad2deg(q_interp2(:,2)), 'DisplayName', 'Segment 2')
plot(t3, rad2deg(q_interp3(:,2)), 'DisplayName', 'Segment 3')
plot(t4, rad2deg(q_interp4(:,2)), 'DisplayName', 'Segment 4')
plot(t5, rad2deg(q_interp5(:,2)), 'DisplayName', 'Segment 5')
plot(t6, rad2deg(q_interp6(:,2)), 'DisplayName', 'Segment 6')

xlabel('Time (s)')
ylabel('Joint Angle (degrees)')
title('Joint 2 Angle')
legend('show')
grid on

subplot(2,1,2)
plot(t1, rad2deg(q_interp1(:,3)), 'DisplayName', 'Segment 1')
hold on
plot(t2, rad2deg(q_interp2(:,3)), 'DisplayName', 'Segment 2')
plot(t3, rad2deg(q_interp3(:,3)), 'DisplayName', 'Segment 3')
plot(t4, rad2deg(q_interp4(:,3)), 'DisplayName', 'Segment 4')
plot(t5, rad2deg(q_interp5(:,3)), 'DisplayName', 'Segment 5')
plot(t6, rad2deg(q_interp6(:,3)), 'DisplayName', 'Segment 6')

xlabel('Time (s)')
ylabel('Joint Angle (degrees)')
title('Joint 3 Angle')
legend('show')
grid on