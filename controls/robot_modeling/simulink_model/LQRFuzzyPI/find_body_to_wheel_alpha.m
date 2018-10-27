%% Tries to find the correct alpha to fit our new body to wheel matrix

t_soccer_period    = 1 / 60.0;  % Control loop in soccer


%% Read data from file
% Ignore first line due to title and the second line since there are some
% extra data
data = dlmread('../../vision-enc-data/enc_vis.txt', ' ', 2, 0);

data_bot_id = data(:, 1);
data_camera_pos = data(:, 2:4);
data_camera_pos(:, 1:2) = data_camera_pos(:, 1:2) / 1000;  % Convert to meters
data_encoder_output = data(:, 5:8) * 2*pi / t_soccer_period / (2048 * 3); % Convert from lsb/ts in hz to meters/s

L = 0.0798576;
r = 0.02768;

thetas = [180 - 30, 180 + 39, 0 - 39, 0 + 30]*pi/180.0 - pi/2;
G = [-sin(thetas(1)), -sin(thetas(2)), -sin(thetas(3)), -sin(thetas(4));
      cos(thetas(1)),  cos(thetas(2)),  cos(thetas(3)),  cos(thetas(4));
                   L,               L,               L,               L];
               
alpha = sym('alpha', 'real');
beta = sym('beta', 'real');

F = [    0, 0, 0;
         0, 0, 0;
     alpha, beta, 0];

BotToWheel = (-1 * G / r)';

alpha_val = -0.036:0.0001:-0.0034; % Best value found at 0.-0.0389
beta_val = -.221:0.0001:-.22;
ERROR_OUT = zeros(length(beta_val), length(alpha_val));
for j = 1:length(beta_val)
    temp = subs(F, beta, beta_val(j));
    for n = 1:length(alpha_val)
        a = alpha_val(n);
        %BTW_temp = double(subs(BotToWheel, alpha, alpha_val(n)));
        WTB_temp = (eye(3,3) - double(subs(temp, alpha, alpha_val(n)))) * pinv(BotToWheel);

        error = 0;
        for i = 1:length(data_bot_id)-1
            if (data_camera_pos(i+1,3) > 3 && data_camera_pos(i,3) < 3)
                continue;
            end

            camera_vel = (data_camera_pos(i+1,:) - data_camera_pos(i,:)) / t_soccer_period;
            vel_diff = WTB_temp * data_encoder_output(i,:)' - camera_vel';
            error = error + sum(vel_diff(3).^2);
        end

        ERROR_OUT(j, n) = error;
    end
end

[val, pos1] = min(min(ERROR_OUT, [], 1));
alpha_val(pos1)
[val, pos2] = min(min(ERROR_OUT, [], 2));
beta_val(pos2)

figure(1);
plot(alpha_val, ERROR_OUT);

figure(2);


temp = subs(F, beta, beta_val(pos2));
WTB_temp = (eye(3,3) - double(subs(temp, alpha, alpha_val(pos1)))) * pinv(BotToWheel);
vel = zeros(1, length(data_bot_id)-1);
for i = 1:length(data_bot_id)-1
    if ((data_camera_pos(i+1,3) > 3 && data_camera_pos(i,3) < 3) || (data_camera_pos(i+1,3) < 3 && data_camera_pos(i,3) > 3))
        continue;
    end

    camera_vel = (data_camera_pos(i+1,:) - data_camera_pos(i,:)) / t_soccer_period;
    vel_diff = WTB_temp * data_encoder_output(i,:)' - camera_vel';
    vel(i) = vel_diff(3);
end

plot(1:length(data_bot_id)-1, vel);