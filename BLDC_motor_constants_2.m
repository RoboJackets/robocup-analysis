%% Maxon EC 45 Flat 42.8mm brushless 70 Watt Motor
% https://www.maxonmotor.com/maxon/view/product/397172

v_nominal = 24;             %Volts
torque_stall = 1.46;    %Nm
current_stall = 39.5;   %A
max_efficiency = 0.85;
no_load_speed = 6110;   %rpm
nominal_speed = 4860;   %rpm
torque_constant = 36.9; %mNm/A
speed_constant = 259;   %rpm/V
mechanical_time_constant = 8.07; %ms -> T_r
rotor_inertia = 0.000181;    %kg/m^2
pole_pairs = 8;         % -> p
number_of_phases = 3;
terminal_inductance = 0.000463;    %H (phase to phase)
terminal_resistance = 0.608;     %Ohms
kf = 0.001;              %This is a random number effectively
theta = 0;
%% Calculated Values
A = ...
    [-terminal_resistance/terminal_inductance, 0, 0;
     0, -terminal_resistance/terminal_inductance, 0;
     0, 0, -kf/rotor_inertia];
 B = ...
     [2/(3 * terminal_inductance), 1/(3 * terminal_inductance), 0;
      -1/(3 * terminal_inductance), -1/(3 * terminal_inductance), 0;
      0, 0, 1/rotor_inertia];
  C = ...
      [1, 0, 0;
       0, 1, 0;
       -1, -1, 0;
       0, 0, 1];