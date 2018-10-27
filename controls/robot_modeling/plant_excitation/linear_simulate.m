addpath('../modeling/');


vars1 = read_excitation('excite_1');
vars2 = read_excitation('excite_2');
vars3 = read_excitation('excite_3_vely');
vars4 = read_excitation('excite_4_vely_4');

u = vars1(:,5:8);
x = vars1(:,1:4);
y = x; % In our case, we know our state variables measurement is encoder feedback
Ts = 1/60;

excite_1_data = iddata(y,u,Ts);
excite_1_data.InputName = {'V1', 'V2', 'V3', 'V4'};
excite_1_data.InputUnit = {'V', 'V', 'V', 'V'};
excite_1_data.OutputName = {'W1', 'W2', 'W3', 'W4'};
excite_1_data.OutputUnit = {'rad/s', 'rad/s', 'rad/s','rad/s'};

excite_1_data = detrend(excite_1_data);

ss1 = ssest(excite_1_data,4);


%plot_excitation('excite_1');

% subplot(2,1,1);
% title('Encoder measurements (rad/s)');
% hold on;
% for i=1:4
%     plot(vars(:,i));
% end
% legend('W1', 'W2', 'W3', 'W4');
% 
% subplot(2,1,2);
% hold on;
% title('Command (v)');
% for i=5:8
%     plot(vars(:,i));
% end
% legend('W1', 'W2', 'W3', 'W4');

hold on;
%plot(vars(:,1)); % Plot wheel 1 encoder readings
%plot(vars(:,1+4)*22); % Plot wheel 1 voltage

% x = [0, 0, 0, 0].'; % assume we start with no vel
% x_hist = x.';
% y_hist = [0, 0, 0, 0];
% 
A = ss5.A;
B = ss5.B;
% C = ss3.C;

for i=1:length(vars1)
% Calculate update to state variables
    row = vars1(i,:);
    input = row(5:8);
    
    x = x + (A*x + B*input.');
    x_hist = [x_hist; x.'];
    %y_hist = [y_hist; (ss5.C*x)'];
    y_hist = [y_hist; x'];
end

hold on;
plot(y_hist(:,1));
plot(vars(:,5));
legend('sim', 'act');

% u = -KK1*xhat-KK2*sigma;
% guess xhat from encoder readings

