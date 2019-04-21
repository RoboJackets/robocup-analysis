function u = inverse_dynamics(v, a, phi)
global Jl
global Jm 
global robot_mass
global robot_radius
global cl
global cm
global G
global gear_ratio
global M
global wheel_radius

    phidot = v(3, 1); %Double check this
    I4 = eye(4);
    GTinv = pinv(G');
    Ginv = pinv(G);
    gRb = rotation_matrix(phi);
    w_l = G' * gRb' * v / wheel_radius;
    
    Rdot = -phidot * [-sin(phi), -cos(phi), 0; cos(phi), sin(phi), 0; 0, 0, 0];
    RTRdot = phidot * [0 1 0; -1 0 0; 0 0 0];
    
    Z = (((Jm + Jl) / gear_ratio^2) * I4) + (wheel_radius^2/gear_ratio^2) * (Ginv * M) * GTinv;
    v = ((cm + cl / gear_ratio^2) * I4) + (wheel_radius^2/gear_ratio^2) * (Ginv * RTRdot) * GTinv;
    
    wldot = (G' * gRb' / wheel_radius) * (a - Rdot * GTinv * wheel_radius * w_l);
    A = -pinv(Z) * v;
    u = Z * gear_ratio * (wldot - A * w_l);
end