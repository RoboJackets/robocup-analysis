function [] = plot_excitation(filename)
%PLOT_EXCITATION Plot plant response
    vars = read_excitation(filename);
    
    subplot(2,1,1);
    title('Encoder measurements (rad/s)');
    hold on;
    for i=1:4
        plot(vars(:,i));
    end
    legend('W1', 'W2', 'W3', 'W4');
    
    subplot(2,1,2);
    hold on;
    title('Command (v)');
    for i=5:8
        plot(vars(:,i));
    end
    legend('W1', 'W2', 'W3', 'W4');

end

