%% Uses a log file to estimate position using our kalman filter

t_soccer_period    = 1 / 60.0;  % Control loop in soccer


%% Read data from file
% Ignore first line due to title and the second line since there are some
% extra data
data = dlmread('../../vision-enc-data/enc_vis.txt', ' ', 2, 0);

data_bot_id = data(:, 1);
data_camera_pos = data(:, 2:4);
data_camera_pos(:, 1:2) = data_camera_pos(:, 1:2) / 1000;  % Convert to meters
data_camera_pos(:, 3) = data_camera_pos(:, 3) - pi/2;
data_encoder_output = data(:, 5:8) * 2*pi / t_soccer_period / (2048 * 3); % Convert from lsb/ts in hz to rad/s


%% Time Grid
time = (0:length(data_bot_id)-1) * t_soccer_period;


%% Robot Constants
thetas = [180 - 30, 180 + 39, 360 - 39, 0 + 30]*pi/180.0;
L = 0.0798576;
J_L = 2.158e-5;
J_m = 1.35e-5;
J = 0.013;
n = 3;
r = 0.02768;
m = 3.678;
c_m = .0001;
c_L = 0.007;
k_m = 0.035;
Rt = 0.978;

% Rad/S = RPM * ??
% RPM = (1 (rev) / 1 (min)) * (2*pi (rad) / 1 (rev)) * (1 (min) / 60 (sec)
rpm_to_rad_p_sec = 2*pi / 60;
EM = 1/(285 * rpm_to_rad_p_sec); % V/RPM, converted to SI units

%% Kinematic Transformation
G = [-sin(thetas(1)), -sin(thetas(2)), -sin(thetas(3)), -sin(thetas(4));
      cos(thetas(1)),  cos(thetas(2)),  cos(thetas(3)),  cos(thetas(4));
                   L,               L,               L,               L];

phi_sym = sym('phi');

gbR = [cos(phi_sym), -sin(phi_sym), 0;
       sin(phi_sym),  cos(phi_sym), 0;
                  0,             0, 1];

M = [m, 0, 0;
    0, m, 0;
    0, 0, J];

Z = (J_m + J_L/(n^2))*eye(4) + ((r^2)/(n^2))*pinv(G)*gbR.'*M*gbR*pinv(G.');
V = (c_m + (c_L/(n^2)))*eye(4) + ((r^2)/(n^2))*pinv(G)*gbR.'*M*diff(gbR, phi_sym)*pinv(G.');

S = (Rt/k_m)*Z;
T = (Rt/k_m)*V + EM*eye(4);

BotToWheel = -1 * G' / r;
WheelToBot = pinv(BotToWheel);

%% State Space Modeling
A_sym = -pinv(S)*T;
B_sym = pinv(S);

A = double(subs(A_sym, phi_sym, 0));
B = double(subs(B_sym, phi_sym, 0));
C = eye(4,4);
D = zeros(4,4);

%% State Feedback Gains
% Q/R for just simple state feedback (-Kx)
Q_fb = eye(4,4);
R_fb = eye(4,4)*2;

robot_ss = ss(A, B, C, D);
[K_fb, ~, e_fb] = lqr(robot_ss, Q_fb, R_fb);

% See word doc for this
% The rotation can be taken out and applied after the fact
% This is so we don't have to descretize a highly nonlinear system
Ao = [zeros(3,3), WheelToBot;
      zeros(4,3),     A-B*K_fb];

% Noise matrices (Unused)
E2 = zeros(3,3);
E1 = zeros(4,4);

Bo = [        E2, zeros(3,4);
      zeros(4,3), K_fb];
  
% The Output matrices must be considered for multiple variations with
% different outputs
%   1. Both Encoders and Camera (For when we have data from both for a
%   single time step
%   2. Encoders (For when we only have the encoder data)
Co_both = [  eye(3,3), zeros(3,4);
           zeros(4,3),          C];
Co_enco = [zeros(4,3),          C];
       
Do_both = zeros(7,7);
Do_enco = zeros(4,7);

full_ss_both = ss(Ao, Bo, Co_both, Do_both);
full_ss_enco = ss(Ao, Bo, Co_enco, Do_enco);
dfull_ss_both = c2d(full_ss_both, t_soccer_period);
dfull_ss_enco = c2d(full_ss_enco, t_soccer_period);

%% Kalman Filter Matrices (Mostly Redefinitions)
% Again, there are 2 version for anything dealing with the kalman filter
% without due to the split kalman filter needed for both camera and encoder
% output and just encoder output
F_k = dfull_ss_both.A; % A
F_k(1:3, 4:end) = t_soccer_period * Ao(1:3, 4:end);
B_k = dfull_ss_both.B; % B
H_k_both = dfull_ss_both.C; % C
H_k_enco = dfull_ss_enco.C;
Q_k = [[.01, 0, 0; 0, .01, 0; 0, 0, 100], zeros(3,4);
       zeros(4,3),   eye(4,4).*100];% Covariance of process noise
R_k_both = [eye(3, 3)*sqrt(.1), zeros(3, 4);
            zeros(4,3), eye(4, 4)*.00001]; % Variance of observation noise
R_k_enco = eye(4, 4)*.00001;

% Position controller
full_X_hat   = zeros( 7, length(data_bot_id) ); % Estimated state vector
camera_Y     = zeros( 3, length(data_bot_id) );  % Camera output
encoder_Y    = zeros( 4, length(data_bot_id) );  % Encoder output

%% Initial Conditions
start_pos = data_camera_pos(1, :)';

full_x_hat        = [start_pos; zeros( 4, 1 )]; % Current position estimate
full_p            = zeros( 7, 7 );            % Current covariance estimate
full_x_hat_camera = [start_pos; zeros( 4, 1 )]; % Estimated pos at last camera update
full_p_camera     = zeros( 7, 7 );            % Covariance at last camera update
camera_y          = start_pos;                  % Current camera reading of pos
encoder_y         = zeros( 4, 1 );              % Current encoder reading of wheel vel

camera_frame_delay = 10; % Number of frames the camera lags behind
encoder_buffer = zeros( 4, camera_frame_delay ); % Past encoder samples before current camera data comes in
u_buffer       = zeros( 3, camera_frame_delay ); % Past wheel output before current camera data comes in
heading_buffer = zeros( 1, camera_frame_delay ); % Past history of the angle


for t = 0:length(data_bot_id)-1	
    rotation_hat = [cos(data_camera_pos(t+1,3)), -sin(data_camera_pos(t+1,3)), 0;
                    sin(data_camera_pos(t+1,3)),  cos(data_camera_pos(t+1,3)), 0;
                                     0,                   0, 1];
    
    % Soccer controller update
    %% Get sensor output
    camera_y    = data_camera_pos(t + 1, :)';
    encoder_y   = data_encoder_output(t + 1, :)';
    commanded_u = zeros(3,1);%rotation_hat * WheelToBot * encoder_y; % Try and guess input from current encoder values

    %% Update Kalman Filter Using Both Camera and Encoder
    % Push current values onto end of queue
    encoder_buffer = [encoder_buffer, encoder_y];
    u_buffer       = [u_buffer, commanded_u];
    heading_buffer = [heading_buffer, full_x_hat(3)];

    % Remove oldest value from queue
    camera_cur  = camera_y;
    wheel_cur   = encoder_buffer(:, 1);
    u_cur       = u_buffer(:, 1);
    heading_cur = heading_buffer(1);
    encoder_buffer(:, 1) = [];
    u_buffer(:, 1)       = [];
    heading_buffer(1) = [];

    % Use both camera and wheel when we have both data for a timestep
    z_k = [camera_cur; wheel_cur];

    % Use state from right before last camera input
    x_hat_k1_k1 = full_x_hat_camera;
    p_k1_k1     = full_p_camera;

    % Robot input
    full_u_kalman = [zeros(3,1); BotToWheel*u_cur];

    % Rotate the part of the A matrix that corresponds with the
    % wheel -> body velocity leaving the integration
    F_k_rot = F_k;
    rot = [cos(data_camera_pos(t+1,3)), -sin(data_camera_pos(t+1,3)), 0;
           sin(data_camera_pos(t+1,3)),  cos(data_camera_pos(t+1,3)), 0;
                          0,                 0, 1];
    F_k_rot(1:3, 4:end) = rot'*F_k(1:3, 4:end); % Need history of angle

    % Predict where we should be this time step
    x_hat_k_k1 = F_k_rot * full_x_hat + B_k * full_u_kalman;
    P_k_k1 = F_k_rot * p_k1_k1 * F_k_rot' + Q_k;

    % Get error between predicted and actual
    y_tilda_k = z_k - H_k_both * x_hat_k_k1;
    S_k = R_k_both + H_k_both * P_k_k1 * H_k_both';

    % Get optimal state estimator gain
    K_k = P_k_k1 * H_k_both' * pinv(S_k);

    % Update state
    x_hat_k_k = x_hat_k_k1 + K_k*y_tilda_k;

    % Calculate the new output and the new covariance
    P_k_k = (eye(7) - K_k * H_k_both) * P_k_k1 * (eye(7) - K_k * H_k_both)' + K_k * R_k_both * K_k';
    y_tilda_k_k = z_k - H_k_both * x_hat_k_k; % (Output is never actually used)

    x_hat_k_k(4:7) = encoder_y;
    full_x_hat = x_hat_k_k;
    full_p = P_k_k;

    full_p_camera = P_k_k;
    full_x_hat_camera = x_hat_k_k;

    %% Update Kalman Filter using Latest Encoder Values (No camera values exist in this period)
    for i = 1:camera_frame_delay
        % Use only the wheel data for the time between the latest
        % camera frame and now
        z_k = encoder_buffer(:, i);

        % Robot Input
        full_u_kalman = [zeros(3,1); BotToWheel*u_buffer(:, i)];

        x_hat_k1_k1 = full_x_hat;
        P_k1_k1 = full_p;

        % Rotate the part of the A matrix that corresponds with the
        % wheel -> body velocity leaving the integration
        F_k_rot = F_k;
        rot = [cos(data_camera_pos(t+1,3)), -sin(data_camera_pos(t+1,3)), 0;
               sin(data_camera_pos(t+1,3)),  cos(data_camera_pos(t+1,3)), 0;
                                    0,                       0, 1];
        F_k_rot(1:3, 4:end) = rot'*F_k(1:3, 4:end); % Need history of angle

        % Predict where we should be this time step
        x_hat_k_k1 = F_k_rot * x_hat_k1_k1 + B_k * full_u_kalman;
        P_k_k1 = F_k_rot * p_k1_k1 * F_k_rot' + Q_k;
        
        % Get error between predicted and actual
        y_tilda_k = z_k - H_k_enco * x_hat_k_k1;
        S_k = R_k_enco + H_k_enco * P_k_k1 * H_k_enco';

        % Get optimal state estimator gain
        K_k = P_k_k1 * H_k_enco' * pinv(S_k);

        % Update state
        x_hat_k_k = x_hat_k_k1 + K_k*y_tilda_k;

        % Calculate the new output and the new covariance
        P_k_k = (eye(7) - K_k * H_k_enco) * P_k_k1 * (eye(7) - K_k * H_k_enco)' + K_k * R_k_enco * K_k';
        y_tilda_k_k = z_k - H_k_enco * x_hat_k_k; % (Output is never actually used)

        x_hat_k_k(4:7) = encoder_y;
        full_x_hat = x_hat_k_k;
        full_p = P_k_k;
    end
    
    %% Store all the results
    % Position controller
    full_X_hat(:, t+1)   = full_x_hat; % Estimated state vector
    camera_Y(:, t+1)     = camera_y;  % Camera output
    encoder_Y(:, t+1)    = encoder_y;  % Encoder output
end

enco_esti = zeros( 3, length(data_bot_id));
enco_esti2 = enco_esti;
enco_esti3 = enco_esti;
enco_esti4 = enco_esti;
cur_est = [start_pos; 0;0;0;0];
cur_est2 = cur_est(1:3);
cur_est3 = cur_est;
cur_est4 = cur_est(1:3);
for n = 1:length(data_bot_id)
    cur_est(3) = camera_Y(3, n);
    rotation_hat = [cos(cur_est(3)), -sin(cur_est(3)), 0;
                    sin(cur_est(3)),  cos(cur_est(3)), 0;
                                  0,                0, 1];
                              
    Ao_rot = Ao;
    Ao_rot(1:3, 4:end) = rotation_hat*Ao_rot(1:3, 4:end);
    cur_est2 = cur_est2(1:3) + 1/60* rotation_hat * WheelToBot * encoder_Y(:, n);
    
    Aor = [zeros(3,3), rotation_hat*WheelToBot;
          zeros(4,3),     A-B*K_fb];
    full_ss_both = ss(Aor, Bo, Co_both, Do_both);
    dfull_ss_both = c2d(full_ss_both, t_soccer_period);
    cur_est3 = dfull_ss_both.A * [cur_est3(1:3); encoder_Y(:, n)];
    
    cur_est = cur_est + t_soccer_period * full_ss_both.A * [cur_est(1:3); encoder_Y(:, n)];
    
    cur_est4 = camera_Y(1:3, n);
    for i = max(1, n - camera_frame_delay):n
        cur_est4 = cur_est4 + t_soccer_period * rotation_hat * WheelToBot * encoder_Y(:, i);
    end
    
    cur_est(3) = wrapToPi(cur_est(3) + pi/2) - pi/2;
    enco_esti(:, n) = cur_est(1:3);
    enco_esti2(:, n) = cur_est2(1:3);
    enco_esti3(:, n) = cur_est3(1:3);
    enco_esti4(:, n) = cur_est4(1:3);
end

f = figure(3);
f.Name =  'Full Encoder Readings';
subplot(411), plot(time, encoder_Y(1,:)), xlabel('t [s]'), ylabel('\omega(t) [rad/s]');
subplot(412), plot(time, encoder_Y(2,:)), xlabel('t [s]'), ylabel('\omega(t) [rad/s]');
subplot(413), plot(time, encoder_Y(3,:)), xlabel('t [s]'), ylabel('\omega(t) [rad/s]');
subplot(414), plot(time, encoder_Y(4,:)), xlabel('t [s]'), ylabel('\omega(t) [rad/s]');

f = figure(4);
f.Name =  'Full Camera Readings';
subplot(311), plot(time, camera_Y(1,:)), xlabel('t [s]'), ylabel('x(t) [m]');
subplot(312), plot(time, camera_Y(2,:)), xlabel('t [s]'), ylabel('y(t) [m]');
subplot(313), plot(time, camera_Y(3,:)), xlabel('t [s]'), ylabel('\theta(t) [rad]');

f = figure(5);
f.Name = 'Full Position Compared (Real Pos, Estimated, Target)';
subplot(311), plot(time, [camera_Y(1,:); full_X_hat(1,:); enco_esti(1,:); enco_esti2(1,:); enco_esti3(1,:); enco_esti4(1,:)]), xlabel('t [s]'), ylabel('x(t) [m]');
legend('Real', 'Kalaman', 'Cont A', 'Raw Cont', 'Disc A', 'Enco Last N');
subplot(312), plot(time, [camera_Y(2,:); full_X_hat(2,:); enco_esti(2,:); enco_esti2(2,:); enco_esti3(2,:); enco_esti4(2,:)]), xlabel('t [s]'), ylabel('y(t) [m]');
legend('Real', 'Kalaman', 'Cont A', 'Raw Cont', 'Disc A', 'Enco Last N');
subplot(313), plot(time, [camera_Y(3,:); full_X_hat(3,:); enco_esti(3,:); enco_esti2(3,:); enco_esti3(3,:); enco_esti4(3,:)]), xlabel('t [s]'), ylabel('\theta(t) [rad]');
legend('Real', 'Kalaman', 'Cont A', 'Raw Cont', 'Disc A', 'Enco Last N');