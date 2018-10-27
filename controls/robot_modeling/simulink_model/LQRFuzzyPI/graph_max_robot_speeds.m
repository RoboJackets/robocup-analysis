thetas = [30, 180 - 30, 180 + 39, 360 - 39]*pi/180.0;
L = 0.0824;
n = 4.091;
r = 0.0285;
G = [-sin(thetas(1)), -sin(thetas(2)), -sin(thetas(3)), -sin(thetas(4));
      cos(thetas(1)),  cos(thetas(2)),  cos(thetas(3)),  cos(thetas(4));
                   L,               L,               L,               L];
BotToWheel = G' / r;
WheelToBot = pinv(BotToWheel);

rpm_to_rad_p_sec = 2*pi / 60;
EM = 1/(285 * rpm_to_rad_p_sec); % V/RPM, converted to SI units

max_voltage = 12;
max_rpm = max_voltage / EM; % Is this right? If not, scale is just going to be off

yaw = 0:0.01:2*pi;
pitch = -pi/2:0.01:pi/2;

out = zeros(3, length(yaw), length(pitch));
out_wheels = zeros(4, length(yaw), length(pitch));

for j = 1:length(pitch)
    for i = 1:length(yaw)
        x = cos(yaw(i))*cos(pitch(j));
        y = sin(yaw(i))*cos(pitch(j));
        w = sin(pitch(j)); % Just to make it easy
        vel = [x;y;w];

        wheels = BotToWheel*vel;
        wheels = wheels ./ max(abs(wheels)) .* max_rpm; % Force the fastest wheel to be max rpm
        max_vel = WheelToBot*wheels;

        out_wheels(:,i, j) = wheels;
        out(:,i, j) = max_vel;
    end
end

for j = 1:length(pitch)
    figure(1);
    plot(out(1, :, j), out(2, :, j));
    axis([-20, 20, -20, 20]);
    title(['Max speeds with omega at ', string(pitch(j))]);
    %figure(2)
    %plot(1:length(yaw), out_wheels(:, :, j));
    pause(0.01)
end