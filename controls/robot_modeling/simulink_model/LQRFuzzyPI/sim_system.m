% Simulates the entire robot, from camera / encoder Kalman filter to robot
% dynamics
%
% Full corresponds to the soccer controller loop while robot corresponds to
% the loop on the robot
%
% Note: Fully self contained in this file, nothing else is needed

clc, clear; rng(0);

%% Time Grid
t_continous_period = 0.0001;
t_robot_period     = 1 / 200.0; % Control loop on robot
t_soccer_period    = 1 / 60.0;  % Control loop in soccer

t_max = 5;

time = 0:t_continous_period:t_max;


%% Robot Constants
thetas = [30, 180 - 30, 180 + 39, 360 - 39]*pi/180.0;
L = 0.0824;
J_L = 2.158e-5;
J_m = 1.35e-5;
J = 0.013;
n = 4.091;
r = 0.0285;
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

BotToWheel = G' / r;
WheelToBot = pinv(BotToWheel);

%% State Space Modeling
A_sym = -pinv(S)*T;
B_sym = pinv(S);

A = double(subs(A_sym, phi_sym, 0));
B = double(subs(B_sym, phi_sym, 0));
C = eye(4,4);
D = zeros(4,4);

%% State Space Integral Gains
% Q/R for the integral state space
Q_int = [  eye(4,4)/10^2,     zeros(4,4);
              zeros(4,4), eye(4,4)*10^2];
R_int = eye(4,4)*5;

robot_ss = ss(A, B, C, D);
[K_int, ~, e_int] = lqi(robot_ss, Q_int, R_int);
drobot_ss = c2d(robot_ss, 1/200);
[K_int_d, ~, e_int_d] = lqi(drobot_ss, Q_int, R_int);

%% PID Constants
% Translational (t) / Rotation (w)
tp = 2;
ti = 0;
td = 0;
wp = 1;
wi = .5;
wd = 0;

pid_sigma = [0;0;0];
pid_previous = [0;0;0];

%% Full State Space Modelling
% This is from the point of view of soccer viewing the robot as the plant
K2 = K_int(:, 5:8);
K1 = K_int(:, 1:4);

% See word doc for this
% The rotation can be taken out and applied after the fact
% This is so we don't have to descretize a highly nonlinear system
Ao = [zeros(3,3), WheelToBot, zeros(3,4);
      zeros(4,3),     A-B*K1,      -B*K2;
      zeros(4,3),          C, zeros(4,4)];

% Noise matrices (Unused)
E2 = zeros(3,3);
E1 = zeros(4,4);

Bo = [        E2, zeros(3,4), zeros(3,4);
      zeros(4,3), zeros(4,4),         E1;
      zeros(4,3),  -eye(4,4), zeros(4,4)];
  
% The Output matrices must be considered for multiple variations with
% different outputs
%   1. Both Encoders and Camera (For when we have data from both for a
%   single time step
%   2. Encoders (For when we only have the encoder data)
Co_both = [  eye(3,3), zeros(3,4), zeros(3,4);
           zeros(4,3),          C, zeros(4,4)];
Co_enco = [zeros(4,3),          C, zeros(4,4)];
       
Do_both = zeros(7,11);
Do_enco = zeros(4,11);

full_ss_both = ss(Ao, Bo, Co_both, Do_both);
full_ss_enco = ss(Ao, Bo, Co_enco, Do_enco);
dfull_ss_both = c2d(full_ss_both, t_soccer_period);
dfull_ss_enco = c2d(full_ss_enco, t_soccer_period);

%% Kalman Filter Matrices (Mostly Redefinitions)
% Again, there are 2 version for anything dealing with the kalman filter
% without due to the split kalman filter needed for both camera and encoder
% output and just encoder output
F_k = dfull_ss_both.A; % A
B_k = dfull_ss_both.B; % B
H_k_both = dfull_ss_both.C; % C
H_k_enco = dfull_ss_enco.C;
Q_k = eye(11, 11); % Covariance of process noise
R_k_both = [eye(3, 3)*sqrt(.01), zeros(3, 4);
            zeros(4,3), eye(4, 4)*.0001]; % Variance of observation noise
R_k_enco = eye(4, 4)*.0001;

%% Prealloc
% Robot wheel speed controller
% Note: Caps represent output to graph variables
%       Lowercase means it contains the current iterations
robot_X_plant = zeros( 4, length(time) ); % Real wheel vels
robot_X_hat   = zeros( 4, length(time) ); % Current estimated wheel vels
robot_U       = zeros( 4, length(time) ); % Input voltage
robot_Y       = zeros( 4, length(time) ); % Same as x
robot_SIGMA   = zeros( 4, length(time) ); % Error integrator
robot_BDY_VEL = zeros( 3, length(time) ); % Body Velocities

% Position controller
full_X_plant = zeros( 11, length(time) ); % Estimated state vector
full_X_hat   = zeros( 11, length(time) ); % Estimated state vector
full_U       = zeros( 3, length(time) );  % Input velocity target
camera_Y     = zeros( 3, length(time) );  % Camera output
encoder_Y    = zeros( 4, length(time) );  % Encoder output

E = zeros( 3, length(time) );
TARGET = zeros(3, length(time) );

%% Initial Conditions
start_pos = [1; 1; pi/2];

robot_x_plant = zeros( 4, 1 );
robot_x_hat   = zeros( 4, 1 );
robot_u       = zeros( 4, 1 );
robot_y       = zeros( 4, 1 );
robot_sigma   = zeros( 4, 1 );

full_x_plant      = [start_pos; zeros( 8, 1 )]; % Real position location
full_x_hat        = [start_pos; zeros( 8, 1 )]; % Current position estimate
full_p            = zeros( 11, 11 );            % Current covariance estimate
full_x_hat_camera = [start_pos; zeros( 8, 1 )]; % Estimated pos at last camera update
full_p_camera     = zeros( 11, 11 );            % Covariance at last camera update
full_u            = zeros( 3, 1 );              % Commanded vel in global frame
camera_y          = start_pos;                  % Current camera reading of pos
encoder_y         = zeros( 4, 1 );              % Current encoder reading of wheel vel

prev_omega = 0;  % Holds the rotation velocity for full nonlinear plant simulation
e = zeros(3, 1); % Position pid error

camera_frame_delay = 10; % Number of frames the camera lags behind
camera_buffer  = repmat( start_pos, 1, camera_frame_delay ); % Delays samples for camera (Not needed in real code)
encoder_buffer = zeros( 4, camera_frame_delay ); % Past encoder samples before current camera data comes in
u_buffer       = zeros( 3, camera_frame_delay ); % Past wheel output before current camera data comes in
heading_buffer = zeros( 1, camera_frame_delay ); % Past history of the angle

prev_y = start_pos;

for t = 0:length(time)-1
    %% Compute Controller Output
    
    % Simple rotation needed to convert world -> robot coordinates
    rotation_real = [cos(full_x_plant(3)), -sin(full_x_plant(3)), 0;
                     sin(full_x_plant(3)),  cos(full_x_plant(3)), 0;
                                        0,                     0, 1];

	
    rotation_hat = [cos(full_x_hat(3)), -sin(full_x_hat(3)), 0;
                    sin(full_x_hat(3)),  cos(full_x_hat(3)), 0;
                                     0,                   0, 1];
    
    % Soccer controller update
    if mod(t*t_continous_period, t_soccer_period) < t_continous_period*.999
        %% Get sensor output
        camera_y    = full_x_plant(1:3) + randn(3,1)*0.01;
        encoder_y   = robot_y;
        commanded_u = rotation_hat*full_u; % Rotate to bot coordinates
         
        %% Update Kalman Filter Using Both Camera and Encoder
        % Push current values onto end of queue
        camera_buffer  = [camera_buffer, camera_y]; % (Not need irl)
        encoder_buffer = [encoder_buffer, encoder_y];
        u_buffer       = [u_buffer, commanded_u];
        heading_buffer = [heading_buffer, full_x_hat(3)];
        
        % Remove oldest value from queue
        camera_cur  = camera_buffer(:, 1);
        wheel_cur   = encoder_buffer(:, 1);
        u_cur       = u_buffer(:, 1);
        heading_cur = heading_buffer(1);
        camera_buffer(:, 1)  = [];
        encoder_buffer(:, 1) = [];
        u_buffer(:, 1)       = [];
        heading_buffer(1) = [];
        
        % Use both camera and wheel when we have both data for a timestep
        z_k = [camera_cur; wheel_cur];
        
        % Use state from right before last camera input
        x_hat_k1_k1 = full_x_hat_camera;
        p_k1_k1     = full_p_camera;
        
        % Robot input
        full_u_kalman = [zeros(3,1); BotToWheel*u_cur; zeros(4, 1)];
        
        % Rotate the part of the A matrix that corresponds with the
        % wheel -> body velocity leaving the integration
        F_k_rot = F_k;
        rot = [cos(heading_cur), -sin(heading_cur), 0;
               sin(heading_cur),  cos(heading_cur), 0;
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
        P_k_k = (eye(11) - K_k * H_k_both) * P_k_k1 * (eye(11) - K_k * H_k_both)' + K_k * R_k_both * K_k';
        y_tilda_k_k = z_k - H_k_both * x_hat_k_k; % (Output is never actually used)
        
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
            full_u_kalman = [zeros(3,1); BotToWheel*u_buffer(:, i); zeros(4, 1)];

            x_hat_k1_k1 = full_x_hat;
            P_k1_k1 = full_p;
            
            % Rotate the part of the A matrix that corresponds with the
            % wheel -> body velocity leaving the integration
            F_k_rot = F_k;
            rot = [cos(heading_buffer(i)), -sin(heading_buffer(i)), 0;
                   sin(heading_buffer(i)),  cos(heading_buffer(i)), 0;
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
            P_k_k = (eye(11) - K_k * H_k_enco) * P_k_k1 * (eye(11) - K_k * H_k_enco)' + K_k * R_k_enco * K_k';
            y_tilda_k_k = z_k - H_k_enco * x_hat_k_k; % (Output is never actually used)

            full_x_hat = x_hat_k_k;
            full_p = P_k_k;
        end
        
        %% Update Output vel
        y = zeros(3,1);
        target = start_pos + [5;-1;1];
        start = start_pos;
        
        t_current = t*t_continous_period;
        t_start = 0;
        
        for i = 1:3
            a_c = 2; % Max Accel
            s_c = 5; % Max Speed
            t_a = s_c / a_c;

            sign = 2*(target(i) > start(i)) - 1;

            t_s = sign*(target(i) - start(i))/s_c - t_a;
            t_end = t_start + t_a + t_s + t_a;
            
            if (t_s < 0)
                t_s = 0;
                t_a = sqrt(abs(target(i) - start(i)) / a_c);%1/2 a t^2 = d
                t_end = t_start + 2 * t_a;
            end

            if (abs(start(i) - target(i)) < 0.01)
                y(i) = start(i);
                continue;
            end

            if t_current < t_start
                y(i) = start(i);
            elseif t_current < t_start + t_a
                y(i) = start(i) + sign*1/2*a_c*(t_current - t_start)^2;
            elseif t_current < t_end - t_a
                y(i) = 1/2*(start(i) + target(i)) + sign*s_c*(t_current - 1/2*(t_start + t_end));
            elseif t_current < t_end
                y(i) = target(i) - sign*1/2*a_c*(t_end - t_current)^2;
            else % t_current > t_end
                y(i) = target(i);
            end
        end
        
        reference_target_vel = (y - prev_y) ./ t_soccer_period;
        prev_y = y;
        
        %% Update PID
        e = y - full_x_hat(1:3);
        pid_sigma = pid_sigma + e * t_soccer_period;
        pid_deriv = (e - pid_previous) ./ t_soccer_period;
        
        pid_vel = zeros(3, 1);
        pid_vel(1:2) = tp * e(1:2) + ti * pid_sigma(1:2) + td * pid_deriv(1:2);
        pid_vel(3) = wp * e(3) + ti * pid_sigma(3) + td * pid_deriv(3);
        
        %% Update Output
        full_u = reference_target_vel + pid_vel;
    end
    
    % Robot controller update
    if mod(t*t_continous_period, t_robot_period) < t_continous_period*.999
        %% Get sensor output
        encoder_tick_per_rev = 1000;
        robot_y = 1 ./ encoder_tick_per_rev .* round(robot_x_plant * encoder_tick_per_rev);
        
        %% Update controller state estimation
        robot_x_hat = robot_y;
        
        targetVel = full_u;
        robot_sigma = robot_sigma + t_robot_period * (BotToWheel*rotation_hat*targetVel - robot_y);
        
        
        %% Update Output
        % State space integral controller
        robot_u = -1 .* (K_int_d * [robot_y; robot_sigma]);
        if any(abs(robot_u) > 12)
            robot_u = robot_u ./ max(robot_u) .* 12; 
        end
    end
    
    %% Store all the results
    % Robot wheel speed controller
    robot_X_plant(:, t+1) = robot_x_plant; % Real wheel vels
    robot_X_hat(:, t+1)   = robot_x_hat; % Current estimated wheel vels
    robot_U(:, t+1)       = robot_u; % Input voltage
    robot_Y(:, t+1)       = robot_y; % Same as x
    robot_SIGMA(:, t+1)   = robot_sigma; % Error integrator
    robot_BDY_VEL(:, t+1) = WheelToBot * robot_x_plant; % Body Velocity

    % Position controller
    full_X_plant(:, t+1) = full_x_plant;
    full_X_hat(:, t+1)   = full_x_hat; % Estimated state vector
    full_U(:, t+1)       = rotation_real*full_u;  % Input velocity target
    camera_Y(:, t+1)     = camera_cur;  % Camera output
    encoder_Y(:, t+1)    = encoder_y;  % Encoder output
    
    E(:, t+1) = e;
    TARGET(:, t+1) = prev_y;
    
    %% Plant Physics    
    % Simulate full nonlinear robot system
    gbRr = [cos(full_x_plant(3)), -sin(full_x_plant(3)), 0;
            sin(full_x_plant(3)),  cos(full_x_plant(3)), 0;
                               0,                     0, 1];
	gbRrd = [-sin(prev_omega), -cos(prev_omega), 0;
              cos(prev_omega), -sin(prev_omega), 0;
                            0,                0, 0];
	
    Z = (J_m + J_L/(n^2))*eye(4) + ((r^2)/(n^2))*pinv(G)*gbRr.'*M*gbRr*pinv(G.');
    V = (c_m + (c_L/(n^2)))*eye(4) + ((r^2)/(n^2))*pinv(G)*gbRr.'*M*gbRrd*pinv(G.');

    S = (Rt/k_m)*Z;
    T = (Rt/k_m)*V + EM*eye(4);

    vel = -pinv(S)*T * robot_x_plant + pinv(S) * robot_u;
    prev_omega = WheelToBot(3,:)*vel;
    
    robot_x_plant = robot_x_plant + t_continous_period*(vel);
    
    % Calculate new position
    full_x_plant(1:3) = full_x_plant(1:3) + t_continous_period * rotation_real' * WheelToBot * robot_x_plant;
    full_x_plant(4:7) = robot_x_plant;
    full_x_plant(8:11) = robot_sigma;
end

f = figure(1);
f.Name = 'Robot Wheel Input Voltage';
subplot(411), plot(time, robot_U(1,:)), xlabel('t [s]'), ylabel('V(t) [V]');
subplot(412), plot(time, robot_U(2,:)), xlabel('t [s]'), ylabel('V(t) [V]');
subplot(413), plot(time, robot_U(3,:)), xlabel('t [s]'), ylabel('V(t) [V]');
subplot(414), plot(time, robot_U(4,:)), xlabel('t [s]'), ylabel('V(t) [V]');

f = figure(2);
f.Name = 'Robot Body Velocity Compared (Real, Target)';
subplot(311), plot(time, [robot_BDY_VEL(1,:); full_U(1,:)]), xlabel('t [s]'), ylabel('v_x(t) [m/s]');
legend('Real', 'Target');
subplot(312), plot(time, [robot_BDY_VEL(2,:); full_U(2,:)]), xlabel('t [s]'), ylabel('v_y(t) [m/s]');
legend('Real', 'Target');
subplot(313), plot(time, [robot_BDY_VEL(3,:); full_U(3,:)]), xlabel('t [s]'), ylabel('\omega(t) [rad/s]');
legend('Real', 'Target');

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
subplot(311), plot(time, [full_X_plant(1,:); full_X_hat(1,:); TARGET(1,:); camera_Y(1,:)]), xlabel('t [s]'), ylabel('x(t) [m]');
legend('Real', 'Estimated', 'Target', 'Camera');
subplot(312), plot(time, [full_X_plant(2,:); full_X_hat(2,:); TARGET(2,:); camera_Y(2,:)]), xlabel('t [s]'), ylabel('y(t) [m]');
legend('Real', 'Estimated', 'Target', 'Camera');
subplot(313), plot(time, [full_X_plant(3,:); full_X_hat(3,:); TARGET(3,:); camera_Y(3,:)]), xlabel('t [s]'), ylabel('\theta(t) [rad]');
legend('Real', 'Estimated', 'Target', 'Camera');

f = figure(6);
f.Name = 'Position Error';
plot(time, E);
legend('X', 'Y', '\theta');
title('Position Error');