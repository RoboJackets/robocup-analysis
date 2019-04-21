global G
global robot_controller
robot_mass = 0.743;       %kg
robot_radius = 0.0789;    %m
gear_ratio = 20/60;
I_robot = 0.5 * robot_mass * robot_radius^2;    %kg.m^2
dt = 0.001;

wheel_radius = 0.029;

%Wheel moment of inertia
Jl = 2.4e-5;
Jm = 135 * 1e-3 * (1e-2)^2;

wheel_angles = [pi * 0.25 pi * 0.75 pi * 1.25 pi * 1.75];

%Geometry Matrix - right now I have added in a transpose that I technically
%dont want - but it makes it work better right now. Double check
%dimensions
G = [-sin(wheel_angles(1)) -sin(wheel_angles(2)) -sin(wheel_angles(3)) -sin(wheel_angles(4));
     cos(wheel_angles(1)) cos(wheel_angles(2)) cos(wheel_angles(3)) cos(wheel_angles(4));
     robot_radius robot_radius robot_radius robot_radius];

%Drag Coefficients
cl = 0.001;
cm = 0;

%Mass Matrix
M = [robot_mass, 0, 0;
     0, robot_mass, 0;
     0, 0, I_robot];
pos = [0; 1; 0];
vel = [1; 0; 1];

robot_controller = controller(dt);


