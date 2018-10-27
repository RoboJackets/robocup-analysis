function parameter_identification()
    % Finds a value near the base value that best matches the test data
    % final_val = base +- alpha*fudge factor, 0 <= alpha <= 1
    params_fudge = struct;
    params_fudge.J_L = 1e-4;
    params_fudge.J_m = 1e-4;
    params_fudge.J   = 1e-1;
    params_fudge.r   = 1e-1;
    params_fudge.m   = 1;
    params_fudge.c_m = 1e-2;
    params_fudge.c_L = 1e-2;

    params = struct;
    params.J_L = 2.158e-5;
    params.J_m = 1.35e-5;
    params.J   = 0.013;
    params.r   = 0.0285;
    params.m   = 3.678;
    params.c_m = .0016;
    params.c_L = 0;
    
    x0 = [params.J_L;
          params.J_m;
          params.J;
          params.r;
          params.m;
          params.c_m;
          params.c_L;];
    
    lb = [params.J_L - params_fudge.J_L;
          params.J_m - params_fudge.J_m;
          params.J   - params_fudge.J;
          params.r   - params_fudge.r;
          params.m   - params_fudge.m;
          params.c_m - params_fudge.c_m;
          params.c_L - params_fudge.c_L];
    % Force lower bound to zero since these cannot be negative
    lb = max(lb, zeros(7,1));
      
    ub = [params.J_L + params_fudge.J_L;
          params.J_m + params_fudge.J_m;
          params.J   + params_fudge.J;
          params.r   + params_fudge.r;
          params.m   + params_fudge.m;
          params.c_m + params_fudge.c_m;
          params.c_L + params_fudge.c_L];
      
    options = optimoptions('fmincon', 'Display', 'iter');
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    func = @(x)get_param_score(x, 0);
    
    x = fmincon(func, x0.', A, b, Aeq, beq, lb.', ub.', [], options)
    get_param_score(x, 1);

end


% Runs the model with the given inputs over the entire data set and returns
% the squared error
% Arguements:
%   param_input: All the parameters that be able to be changed
%       [1] = J_L
%       [2] = J_m
%       [3] = J
%       [4] = r
%       [5] = m
%       [6] = c_m
%       [7] = c_L
% Return:
%   Squared error over all data sets

function [sq_error] = get_param_score(param_input, plot_res)
    addpath('../modeling/');
    
    [in1, out1] = read_excitation('excite_1');
    [in2, out2] = read_excitation('excite_2');
    [in3, out3] = read_excitation('excite_3_vely');
    [in4, out4] = read_excitation('excite_4_vely_4');

    in = [in1; in2; in3; in4];
    out = [out1; out2; out3; out4];
    
    params = struct;
    params.thetas = [30, 180 - 30, 180 + 39, 360 - 39]*pi/180.0;
    params.L = 0.0824;
    params.J_L = param_input(1);
    params.J_m = param_input(2);
    params.J = param_input(3);
    params.n = 4.091;
    params.r = param_input(4);
    params.m = param_input(5);
    params.c_m = param_input(6);
    params.c_L = param_input(7);
    params.k_m = 0.035;
    params.Rt = 0.978;
    params.EM = 1/285;

    [A, B] = get_control_matrices(params, 0);

    x = [0, 0, 0, 0].';
    x_out = zeros(length(out), 4);
    sq_error = 0;

    for i = 1:length(in)

        error = out(i,:) - x.';
        sq_error = sq_error + sum(error .* error);

        input = in(i, :);
        x = A*x + B*input.';
        x_out(i, :) = x.';
    end
    
    if (plot_res)
        figure(1);
        hold on
        plot(1:length(out), x_out(:,1));
        plot(1:length(out), out(:,1));
        hold off
    end
end