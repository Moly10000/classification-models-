t = linspace(0, 10, 1000);

goal_x = -10;
goal_y = -3;
x0 = 0;
y0 = 0;
theta0 = 0;
state = [x0;y0;theta0];
pose = lsode("robotdifferentialintegral", state, t);

figure(1);
plot(pose(:,1), pose(:,2));
hold on;
plot(goal_x, goal_y, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X [m]');
ylabel('Y [m]');
title('Robot Trajectory with P and PI Controllers');
grid on;
legend('Robot Trajectory', 'Goal');
