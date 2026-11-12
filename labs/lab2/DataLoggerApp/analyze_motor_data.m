close all; clear; clc;

%% 1. Loading the raw data from the TurtleBot
load('motor_data.mat'); % Load the 'data' structure saved by the data logger

% Check that some samples have been acquired
assert(data.size > 0, 'No samples found in the .mat file');

% Vector extraction, same as in runSerialDataLoggerApp.m
t  = data.time - data.time(1);   % Time vector aligned to 0 s
w1 = data.signals{1}(1,:);       % Angular speed of motor 1 (raw)
w2 = data.signals{1}(2,:);       % Angular speed of motor 2 (raw)
u1 = data.signals{2}(1,:);       % Voltage/Duty applied to motor 1
u2 = data.signals{2}(2,:);       % Voltage/Duty applied to motor 2

% Automatic computation of the sampling period Ts (it should be 0.01 s)
%Ts = mean(diff(t));
Ts = 0.01;

%% 2. Low-Pass Filter design (from filter_example.m)
fc = 2.0; % Cut-off frequency [Hz] -> TO BE TUNED by trial and error

% Continuous transfer function H(s) = (2*pi*fc) / (s + 2*pi*fc)
sysF = tf(2*pi*fc, [1, 2*pi*fc]);

% Discretization with the Tustin method
sysF_d = c2d(sysF, Ts, 'tustin');
[numF, denF] = tfdata(sysF_d, 'v');

%% 3. Applying the filter to the angular speeds
w1_filt = filter(numF, denF, w1);
w2_filt = filter(numF, denF, w2);

%% 4. Saving into the new .mat file
save('motor_data_filtered.mat', 't', 'w1', 'w2', 'w1_filt', 'w2_filt', 'u1', 'u2', 'fc', 'Ts');
fprintf('File saved successfully: motor_final_filtered.mat (fc = %.2f Hz)\n', fc);

%% 5. Visual check (comparison of the two filtered speeds)
figure('Color', 'w', 'Name', 'Filtered speeds TurtleBot');
ax = gca;

plot(t, w1, 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2); hold on;
plot(t, w2, 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2);
h1f = plot(t, w1_filt, 'b', 'LineWidth', 1.8);
h2f = plot(t, w2_filt, 'r', 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Angular speed [rad/s]');
title(ax, sprintf('Speeds w_1 and w_2: raw vs filtered (f_c = %.2f Hz)', fc));
legend('w_1 raw', 'w_2 raw', 'w_1 filtered', 'w_2 filtered', 'Location', 'best'); grid on;

% fc slider: recomputes the filter and updates the filtered curves
min_slider_value  = 0.1;
max_slider_value  = 20.0;
step_slider_value = 0.1; % (Hz per notch)

fc_step = step_slider_value / (max_slider_value - min_slider_value);
uicontrol('Style', 'slider', 'Min', min_slider_value, 'Max', max_slider_value, 'Value', fc, ...
    'SliderStep', [fc_step 10*fc_step], ...
    'Units', 'normalized', 'Position', [0.15 0.02 0.7 0.04], ...
    'Callback', @(src, ~) updateFilterPlot(src.Value, w1, w2, h1f, h2f, ax, Ts, min_slider_value, step_slider_value));

%% 6. Applied inputs u1 and u2 (separate window)
figure('Color', 'w', 'Name', 'Inputs TurtleBot');
plot(t, u1, 'b', 'LineWidth', 1.5); hold on;
plot(t, u2, 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Voltage [V]');
title('Applied inputs u_1 and u_2');
legend('u_1', 'u_2', 'Location', 'best'); grid on;

%% 7. Derivatives of w1 and w2 (separate window)
figure('Color', 'w', 'Name', 'Speed derivatives TurtleBot');
plot(t, gradient(w1, Ts), 'Color', [0.6 0.75 1.0], 'LineWidth', 1.0); hold on;
plot(t, gradient(w2, Ts), 'Color', [1.0 0.65 0.65], 'LineWidth', 1.0);
plot(t, gradient(w1_filt, Ts), 'b', 'LineWidth', 1.8);
plot(t, gradient(w2_filt, Ts), 'r', 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Angular acceleration [rad/s^2]');
title(sprintf('Derivatives of w_1 and w_2: raw vs filtered (f_c = %.2f Hz)', fc));
legend('dw_1/dt raw', 'dw_2/dt raw', 'dw_1/dt filtered', 'dw_2/dt filtered', 'Location', 'best'); grid on;

function updateFilterPlot(fcNew, w1, w2, h1f, h2f, ax, Ts, fc_min, fc_step_hz)
    fcNew = fc_min + round((fcNew - fc_min) / fc_step_hz) * fc_step_hz;
    sysF = tf(2*pi*fcNew, [1, 2*pi*fcNew]);
    sysF_d = c2d(sysF, Ts, 'tustin');
    [numF, denF] = tfdata(sysF_d, 'v');
    h1f.YData = filter(numF, denF, w1);
    h2f.YData = filter(numF, denF, w2);
    title(ax, sprintf('Speeds w_1 and w_2: raw vs filtered (f_c = %.2f Hz)', fcNew));
end
