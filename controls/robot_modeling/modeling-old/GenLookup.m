robot_params = RobotParams;

robot_params.M_bot = 2.205;
robot_params.I_bot = 0.00745879949;
robot_params.g = 1.0/3.0;
robot_params.r = 0.0285623;
robot_params.L = 0.0798576;
robot_params.Rt = 0.464;
robot_params.K_e = 30.0/(380*pi);
robot_params.K_t = 0.0251;
robot_params.K_f = 0.0001;
robot_params.I_asm = 2.43695253e-5;
robot_params.V = 18;
robot_params.wheel_angles = [30, 180 - 30, 180 + 39, 360 - 39]*pi/180.0;

J = robot_params.J;
G = robot_params.G;
B = robot_params.B;
A_1 = robot_params.A_1;
A_2 = robot_params.A_2;

MinW = -pi/2;
MaxW = pi/2;
NumEntries = 100;

rot_vels = linspace(MinW, MaxW, NumEntries);

trans_vel_weight = 1;
rot_vel_weight = 0.1;
u_weight = 1.0;

Q = [trans_vel_weight,                    0,              0;
                    0,     trans_vel_weight,              0;
                    0,                    0, rot_vel_weight];
R = eye(4) * u_weight;

fid = fopen('LQRLookup.cpp', 'w');

fprintf(fid, '// Do not modify this file, it is automatically generated at compile time\n');
fprintf(fid, '#include \"LQRLookup.hpp\"\n');
fprintf(fid, '\n');

fprintf(fid, 'const float MinW_rad_per_sec = %f;\n', MinW);
fprintf(fid, 'const float MaxW_rad_per_sec = %f;\n', MaxW);
fprintf(fid, 'const float NumEntries = %d;\n', NumEntries);
fprintf(fid, '\n');

fprintf(fid, 'const float LQRLookupValues[] = {\n');

for w=rot_vels
    A = A_1 + A_2 * w;
    [K, S, E] = lqr(A, B, Q, R);
    fprintf(fid, '    // rot vel = %f\n', w);

    for i=1:size(K,1)
        fprintf(fid, '    ');
        for j=1:size(K,2)
            fprintf(fid, ' %f,', K(i,j));
        end
        fprintf(fid, '\n');
    end
end
fprintf(fid, '};\n');

fclose(fid);
