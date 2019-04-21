%Run body_controller before this program
classdef controller
    properties
        integral
        dt
        kp
        ki
        kd
    end
    methods
        function obj = controller(dt)
            obj.dt = dt;
            obj.integral = zeros(3,1);
            obj.kp = 4;
            obj.ki = 0.0;
            obj.kd = 1;
        end
        function u = control(obj, x, v, rx, rv, ra)
%             global Jl
%             global Jm 
%             global robot_mass
%             global robot_radius
%             global cl
%             global cm
            global G
            gRb = rotation_matrix(x(2));
            gRp = rotation_matrix(rx(2));
            error_path_relative = gRp * (rx - x);
            obj.integral = obj.integral + (obj.dt * [1 0 0; 0 1 0; 0 0 1e-2] \ error_path_relative);
            
            Ginv = pinv(G);
            error = gRb' * (rx - x);
            error(3, 1) = error(3, 1) * 0.1;
            
            derivative = gRb' * (rv - v);
            derivative(3, 1) = derivative(3, 1) * 0.01;
            
            uff = inverse_dynamics(rv, ra, x(2));%call inverse_dynamics
            up = obj.kp * Ginv * error;
            ui = obj.ki * Ginv * gRb * gRp' * obj.integral;
            ud = obj.kd * Ginv * derivative;
            
            u = uff + up + ui + ud;
        end
    end
    methods(Static)
        function reset()
            obj.integral = zeros(3,1);
        end
    end
end