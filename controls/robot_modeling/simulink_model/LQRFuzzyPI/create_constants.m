%% Constants
X_0 = [0, 0, 0, 0];

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
w_n = 1/10;

% Rad/S = RPM * ??
% RPM = (1 (rev) / 1 (min)) * (2*pi (rad) / 1 (rev)) * (1 (min) / 60 (sec)
rpm_to_rad_p_sec = 2*pi / 60;
EM = 1/(285 * rpm_to_rad_p_sec); % V/RPM, converted to SI units


%% State Space Matrices / Kinematic Translations
G = [-sin(thetas(1)), -sin(thetas(2)), -sin(thetas(3)), -sin(thetas(4));
    cos(thetas(1)),  cos(thetas(2)),  cos(thetas(3)),  cos(thetas(4));
    L,            L,            L,           L];

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

A_sym = -pinv(S)*T;
B_sym = pinv(S);

% After taking derivative of gbR, phi_sym becomes phi_sym_dot, which we
% substitute in 0 for because we linearize around 0 rotation velocity.
A = double(subs(A_sym, phi_sym, 0));
B = double(subs(B_sym, phi_sym, 0));
C = eye(4,4);
D = zeros(4,4);

wvR = pinv(G.')*r;

BotToWheel = G' / r;
WheelToBot = pinv(BotToWheel);

%% State Space Integral Gains
% Q/R for the integral state space
Q_int = [  eye(4,4)/10^2, zeros(4,4);
     zeros(4,4), eye(4,4)*10^2];
R_int = eye(4,4)*5;

s = ss(A, B, C, D);
[K_int, ~, e_int] = lqi(s, Q_int, R_int);
so = c2d(s, 1/200);
[K_int_d, ~, e_int_d] = lqi(so, Q_int, R_int);

%% State Feedback Gains
% Q/R for just simple state feedback (-Kx)
Q_fb = eye(4,4);
R_fb = eye(4,4)*2;
[K_fb, ~, e_fb] = lqr(s, Q_fb, R_fb);


%% Camera Delay Properties for Smith Predictor
% Camera delay and our estimated delay
delay = 100;
delay_est = delay * 1;

%% PID Constants
% Translational (t) / Rotation (w)
tp = 0;
ti = 0;
td = 0;
wp = 0;
wi = 0;
wd = 0;

%% Descrete Time Model

% Camera Properties
delay_sample = 0;
camera_noise = 0.000001;
camera_ts = 0.01;

% Encoder Buffer / Resolution
buffer_size = 1;
encoder_ts = 0.01;


K2 = K_int(:, 5:8);
K1 = K_int(:, 1:4);

% See word doc for this
Ao = [zeros(3,3), gbR*pinv(G')*r/n, zeros(3,4);
      zeros(4,3),           A-B*K1,      -B*K2;
      zeros(4,3),                C, zeros(4,4)];
Ao = double(subs(Ao, phi_sym, 0));

E2 = zeros(3,3);
E1 = zeros(4,4);

Bo = [        E2, zeros(3,4), zeros(3,4);
      zeros(4,3), zeros(4,4),         E1;
      zeros(4,3),  -eye(4,4), zeros(4,4)];
  
  
Co_both = [  eye(3,3), zeros(3,4), zeros(3,4);
           zeros(4,3),          C, zeros(4,4)];

Co_enco = [zeros(4,3),          C, zeros(4,4)];
       
Do_both = zeros(7,11);
Do_enco = zeros(4,11);
  

%ssi = ss(Ao(4:end, 4:end), Bo(4:end, 4:end), Co(4:end, 4:end), Do(4:end, 4:end));
ssa = ss(Ao, Bo, Co_both, Do_both);
ssb = ss(Ao, Bo, Co_enco, Do_enco);
ssad = c2d(ssa, encoder_ts);
ssbd = c2d(ssb, encoder_ts);

%% Kalman Filter
F_k = ssad.A; % A
B_k = ssad.B; % B
H_k_both = ssad.C; % C
H_k_enco = ssbd.C;
Q_k = eye(11, 11); % Covariance of process noise
R_k_both = [eye(3, 3), zeros(3, 4);
            zeros(4,3), eye(4, 4)]; % Variance of observation noise
R_k_enco = eye(4, 4);

