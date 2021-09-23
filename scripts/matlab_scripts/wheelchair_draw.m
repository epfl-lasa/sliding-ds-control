function wheel_points = wheelchair_draw(robot_radius,robot_center,tilted_angle)

if nargin < 3
    tilted_angle = 0;
end

R = [cos(tilted_angle) -sin(tilted_angle);sin(tilted_angle) cos(tilted_angle)];

x = [-0.7 -0.7 -0.5 -0.5 0.1 0.1 0.5 0.5 0.7 0.7 0.5 0.5];
y1 = [0 0.5 0.5 0.7 0.7 0.5 0.5 0.4 0.4 0.2 0.2 0];
y2 = -y1;
x = [x fliplr(x)];
y = [y1 fliplr(y2)];

wheel_points = robot_radius*[x;y];
wheel_points = R*wheel_points + robot_center;
end