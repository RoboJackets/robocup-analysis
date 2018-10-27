addpath('../plant_excitation/');

l = 5;
d = 0.089;
r = 0.0325;

R = 4.3111;
M = 2.34;
J = 0.0169;
Bv = 0.4978;
Bvn = 0.6763;
Bw = 0.0141;

Cv = 1.8738;
Cvn = 2.2198;
Cw = 0.1385;
Cv = 0;
Cvn = 0;
Cw = 0;

Kt = 25.1 / 1000; % Nm/A
Kt = .003;

% This may be wrong, see https://en.wikipedia.org/wiki/Motor_constants
Kv = 50; %Kt * 2*pi/60;
Kv = 

A11 = (-2*Kt^2*l^2)/(r^2*R*M) - Bv/M;
A22 = (-2*Kt^2*l^2)/(r^2*R*M) - Bvn/M;
A33 = (-4*d^2*Kt^2*l^2)/(r^2*R*J) - Bw/J;

A = [A11 0 0;
     0 A22 0;
     0 0 A33];

B = (l*Kt)/(r*R) * [0 -1/M 0 1/M;
                    1/M 0 -1/M 0;
                    d/J d/J d/J d/J];
                
K = [-Cv/M 0 0;
    0 -Cvn/M 0;
    0 0 -Cw/J];

% x(t) = [v(t) vn(t) w(t)]'
% x_dot(t) = A*x(t)+B*u(t)+K*sgn(x)

WheelAngles = [180-30, 180+39, 360-39, 0+30];
WheelAngles = WheelAngles * pi/180.0;
WheelDist = 0.0798576;
WheelRadius = 0.02768;

BotToWheel = [-sin(WheelAngles(1)) cos(WheelAngles(1)) WheelDist;
              -sin(WheelAngles(2)) cos(WheelAngles(2)) WheelDist;
              -sin(WheelAngles(3)) cos(WheelAngles(3)) WheelDist;
              -sin(WheelAngles(4)) cos(WheelAngles(4)) WheelDist];
    
BotToWheel = BotToWheel * -1;
BotToWheel = BotToWheel / WheelRadius;

WheelToBot = pinv(BotToWheel);

vars = read_excitation('excite_4_vely_4');

x = [0 0 0].'; % assume we start with no vel
x_hist = x.';

x_hist_real = [0 0 0];

for i=1:length(vars)
% Calculate update to state variables
    row = vars(i,:);
    input = row(5:8);
    % input = [0 0 0 0];
    
    x = x + (A*x + B*input.' + K*sign(x)) * (1/60);
    % x = x + (A*x + B*input.') * (1/60);
    % x(1)
    x_hist = [x_hist; x.'];
    x_hist_real = [x_hist_real; (WheelToBot*input')'];
end

for i=1:3
    subplot(3,1,i);
    hold on;
    plot(x_hist(:,i));
    plot(x_hist_real(:,i));
end