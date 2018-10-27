function [ t, Ts, input_v, wheel_vels ] = read_excitation(filename)
%READ_EXCITATION Reads cleaned csv of plant excitation data
    
    vals = csvread(filename);
    
    % from ControllerTaskThread, convert from FPGA to real time
    unpack_time_constant = (1.0 / 18.432e6) * 2 * 128;
    unpack_enc_constant = 2.0 * pi / (2048 * 3);
    % assuming 19.0 volts average and range of -512 to 512
    unpack_cmd_volts = (1.0 / 1024) * 19.0;
    
    % print_line_num = 2100;
    % vals(print_line_num,:)
    
    vals(:,9) = vals(:,9) * unpack_time_constant;
    
    for i=1:4
        vals(:,i) = vals(:,i) * unpack_enc_constant ./ vals(:,9);
    end
    
    for i=5:8
        vals(:,i) = vals(:,i) * unpack_cmd_volts;
    end

    % vals(print_line_num,:)
    
    input_v = vals(:,5:8);
    wheel_vels = vals(:,1:4);
    
    Ts = 1/60;
    t = 0:Ts:(length(input_v)*Ts - Ts);
end

