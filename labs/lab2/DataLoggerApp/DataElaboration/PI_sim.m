close all; clear; clc;

% --- SYSTEM PARAMETERS ---
% values estimated on motor 1 at Va = 3.84 V (63.2 % method), then used to
% design the controller in open loop with Ziegler-Nichols.
Kp = 1.6196;  % proportional gain of the PI controller
Ki = 8.9979;  % integral gain of the PI controller [V/(rad/s)/s]
tau = 0.06;   % pure time delay of the motor model [s]
mu = 2.4080;  % motor static gain [rad/s per V]
T = 0.26;     % motor time constant [s]
Ts = 0.01;    % sample time (10 ms based on STM32 config)

% --- SIMULATION SETUP ---
total_time = 5;    % simulate for 5 seconds
N = total_time / Ts; % number of samples
t = (0:N-1) * Ts;  % time vector [s]

w_target = 10; % desired target velocity (10 rad/s)

% initialize memory arrays
w_measured      = zeros(1, N); % motor speed at each sample [rad/s]
applied_voltage = zeros(1, N); % voltage commanded by the PI after the clamp [V]
error_array     = zeros(1, N); % speed error, stored only for inspection

% static variable for the integral
I = 0; % accumulated integral term [V], keeps memory between samples

% pre-padded array, in this way we implement the delay exp(-tau * s)
n_delay = round(tau/Ts);        % delay expressed in samples (tau / Ts)
Va_hist = zeros(1, N + n_delay); % voltage history; zeros at the start: motor at rest

V_max = 6.0; % maximum voltage allowed by the driver (50 % of V_PSU)

% --- CLOSED-LOOP SIMULATION CYCLE ---
for k = 1:N-1

    % acquire current velocity
    w_current = w_measured(k); % speed at this sample [rad/s]

    % calculate error (target - measured)
    error = w_target - w_current;
    error_array(k) = error;

    % PI controller (translation of the C code)
    P = Kp * error;             % proportional term [V]
    I = I + Ki * error * Ts;    % integral term with memory [V]
    Va = P + I;                 % voltage calculated by the controller [V]

    %{
    % version with controlled update of the accumulator
    Va_unsat = P + I;                        % 1. hypothetical output
    I_new    = I + Ki * error * Ts;          % 2. hypothetical integral

    % 3. accumulate only if it does not push further into saturation
    condition1 = Va_unsat >= -V_max && Va_unsat <= V_max;
    condition2 = Va_unsat > 0 && error < 0;
    condition3 = Va_unsat < 0 && error > 0;
    if (condition1) || (condition2) || (condition3)
        I = I_new;
    end

    Va = max(-V_max, min(V_max, P + I));     % 4. real output (still clamped)
    %}

    % anti-windup / saturation: the motor cannot exceed the supply voltage
    if Va > V_max
        Va = V_max;
    elseif Va < -V_max
        Va = -V_max;
    end

    applied_voltage(k) = Va;
    Va_hist(k + n_delay) = Va; % store with an offset to create the delay

    % physical motor model (plant)
    % calculate how the motor reacts to the applied voltage Va.
    % discrete formula (forward Euler) based on G(s) = mu / (1 + T*s)
    w_measured(k+1) = w_measured(k) + (Ts / T) * (-w_measured(k) + mu * Va_hist(k));

end

% --- plot 1: Closed-loop PI simulation
figure('Color', 'w', 'Name', 'Closed-loop PI simulation');
sgtitle('Closed-loop PI simulation');
subplot(2,1,1);
plot(t, w_measured, 'b', 'LineWidth', 1.5); hold on;
yline(w_target, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (rad/s)');
title('Closed-Loop Motor Response');
legend('Measured Velocity', 'Target Velocity');
grid on;

subplot(2,1,2);
plot(t, applied_voltage, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Voltage (V)');
title('Control Action (PI Output Voltage)');
grid on;

