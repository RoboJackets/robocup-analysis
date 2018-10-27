Rt = 0.978;
k_m = 33.5 / 1000;
EM = (1/285) / 60;
J_L = 2.158e-5;
J_m = 1.35e-5;
J = 0.013;
l = 0.0824;
m = 3.678;
n = 4.091;
r = 0.0285;
c_m = 0.0015;
c_L = 0;


k1 = J_m + J_L/n^2 + r^2/n^2*(0.2810*m + 11.75*J);
k2 = r^2/n^2*(0.03812*m + 11.75*J);
k3 = r^2/n^2*(-0.2619*m + 9.050*J);
k4 = r^2/n^2*(-0.05717*m+9.050*J);
k5 = J_m + J_L/n^2 + r^2/n^2*(0.2459*m + 6.9710*J);
k6 = r^2/n^2*(0.07323*m + 6.9710*J);
k7 = c_m + c_L/n^2;
k8 = 0.2784*r^2/n^2*m;
k9 = 0.02184*r^2/n^2*m;
k10 = 0.2566*r^2/n^2*m;
k11 = 0.2347*r^2/n^2*m;

psi_dot = 0; %X_dot_b(3);

Z = [k1, k2, k3, k4;
     k2, k1, k4, k3;
     k3, k4, k5, k6;
     k4, k3, k6, k5];
 
V = [          k7, -k8*psi_dot,   k9*psi_dot,  k10*psi_dot;
       k8*psi_dot,          k7, -k10*psi_dot,  -k9*psi_dot;
      -k9*psi_dot, k10*psi_dot,           k7, -k11*psi_dot;
     -k10*psi_dot,  k9*psi_dot,  k11*psi_dot,           k7];

% E = (Rt/k_m Z) \omega_m_dot + (Rt/k_m V + EM.I_4x4)\omega_m
% E - (Rt/k_m V + EM.I_4x4)\omega_m = (Rt/k_m Z) \omega_m_dot
% omega_m_dot = (Rt/k_m Z)^-1 (E - (Rt/k_m V + EM.I_4x4) omega_m)
B = pinv(Rt / k_m * Z);
A = -1*((Rt / k_m * Z)\(Rt / k_m * V + EM * eye(4,4)));
