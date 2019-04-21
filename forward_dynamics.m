function [dxdt, wldot] = forward_dynamics(x, u, phi)
global Jl
global Jm 
global robot_mass
global robot_radius
global wheel_radius
global cl
global cm
global G
global gear_ratio
global M
    phidot = x(3, 1); %Double check this
    I4 = eye(4);
    GTinv = pinv(G');
    Ginv = pinv(G);
    gRb = rotation_matrix(phi);
    w_l = G' * gRb' * x / robot_radius;
    
    Rdot = -phidot * [-sin(phi), -cos(phi), 0; cos(phi), sin(phi), 0; 0, 0, 0];
    RTRdot = phidot * [0 1 0; -1 0 0; 0 0 0];
    
    Z = (((Jm + Jl) / gear_ratio^2) * I4) + (wheel_radius^2/gear_ratio^2) * Ginv * M * GTinv;
    V = ((cm + cl / gear_ratio^2) * I4) + (wheel_radius^2/gear_ratio^2) * Ginv * RTRdot * GTinv;
    
    B = pinv(Z)/gear_ratio;
    A = pinv(Z) * V;
    wldot = A * w_l + B * u;
    
    dxdt = Rdot * GTinv * wheel_radius * w_l + gRb * GTinv * wheel_radius * wldot;
end
