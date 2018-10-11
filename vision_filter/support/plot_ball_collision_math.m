figure(1);
robot_pos = [-.12, 4];
ball_pos_safe = [-.887, 3.467];
ball_pos = [-0.03, 3.98];
intersect_points = [0.0289, 4.017; -0.205, 3.876];

ip_b = [intersect_points(2,:); intersect_points(2,:) + [-0.68245, -0.4094]];
r_ip = [robot_pos; [-0.085, -0.123] + robot_pos];

proj = [intersect_points(2,:); [-0.413, -.5961] + intersect_points(2,:)];

diff = [-0.269, .1866];
diff = [-diff + proj(2,:); diff + proj(2,:)];

refl = [intersect_points(2,:); [-0.144, -0.782] + intersect_points(2,:)];

ssm = [1, 0; 0, 1];
ast = [-.5961, 0.413; -0.413, -.5961];
ast = ast ./ norm(ast);
iast = ast';

reflected = ast*[-0.144, -0.782]';
theta = acos(dot([-0.144, -0.782], [-0.085, -0.123]) / (norm([-0.144, -0.782])*norm([-0.085, -0.123])));
if (reflected(1) > 0)
    theta = -theta;
end
t = min(theta, pi/2 - theta)*0.5;
asm = [cos(t), -sin(t);
       sin(t),  cos(t)];

ft = iast * ssm*asm * ast * [-0.144, -0.782]' / sqrt(sum([-0.144, -0.782].^2));
ft2 = ft + intersect_points(2,:)';

plot(robot_pos(1), robot_pos(2), 'o');
hold on
plot(ball_pos_safe(1), ball_pos_safe(2), 'o')
plot(ball_pos(1), ball_pos(2), 'o')
plot(intersect_points(:,1), intersect_points(:,2))
plot(ip_b(:,1), ip_b(:,2))
plot(r_ip(:,1), r_ip(:,2))
plot(proj(:,1), proj(:,2))
plot(diff(:,1), diff(:,2))
plot(refl(:,1), refl(:,2))
plot(ft2(1), ft2(2), 'o')
hold off

legend({'Robot Pos', 'Ball Safe Pos', 'Ball Pos', 'Intersect Pts', 'ip_b', 'r_ip', 'projection', 'diff', 'refl', 'final vel vector'}, 'Location','southeast')

daspect([1 1 1])