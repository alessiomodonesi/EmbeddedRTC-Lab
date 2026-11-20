close all; clear; clc;

input_file = 'motor_data_forward_filtered.mat';
load(input_file);
w_real = w1_filt; % raw measured speed of motor 1 FILTERED [rad/s]


% --- SYSTEM PARAMETERS ---
% values estimated on motor 1 at Va = 3.84 V (63.2 % method), then used to
% design the controller in open loop with Ziegler-Nichols.
Kp = 1.6196;  % proportional gain of the PI controller
Ki = 8.9979;  % integral gain of the PI controller [V/(rad/s)/s]
tau = 0.06;   % pure time delay of the motor model [s]
mu = 2.4080;  % motor static gain [rad/s per V]
T = 0.26;     % motor time constant [s]
Ts = 0.01;    % sample time (10 ms based on STM32 config)

%% --- OPEN-LOOP SIMULATION: theoretical models G(s) and P(s), N controller ---
% input = the voltage steps actually recorded, not the controller output
t_data = t - t(1);                     % time vector of the recording, aligned to 0 s
u_data = u1;                           % commanded voltage of motor 1
u_data = max(-6.0, min(6.0, u_data));  % same 50 % duty limit of the driver

N = numel(t_data);                 % number of samples in the recording
w_G = zeros(1, N+1);               % simulated speed, model G(s) = mu/(1+Ts)
w_P = zeros(1, N+1);               % simulated speed, model P(s) = G(s)*exp(-tau*s)
n_delay = round(tau/Ts);           % delay expressed in samples (tau / Ts)
u_hist = zeros(1, N + n_delay);    % delay line for P(s)

for k = 1:N
    u_hist(k + n_delay) = u_data(k);

    w_G(k+1) = w_G(k) + (Ts/T) * (-w_G(k) + mu*u_data(k));   % N delay
    w_P(k+1) = w_P(k) + (Ts/T) * (-w_P(k) + mu*u_hist(k));   % input delayed by tau
end

% --- plot 1: input actually fed to the two models
figure('Color', 'w', 'Name', 'Input voltage');
plot(t_data, u_data, 'k', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Voltage [V]');
title('Recorded voltage steps (input of the open-loop simulation)');
grid on;

% --- plot 2: models vs the measured speed
figure('Color', 'w', 'Name', 'Models vs measured speed');
plot(t_data, w_real, 'Color', [0.7 0.7 0.7], 'LineWidth', 1.0); hold on;
plot(t_data, w_G(1:N), 'b', 'LineWidth', 1.5);
plot(t_data, w_P(1:N), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Velocity [rad/s]');
title('Open-loop: theoretical G(s), P(s) vs measured speed');
legend('w_1 measured', 'G(s)', 'P(s)', 'Location', 'best');
grid on;
