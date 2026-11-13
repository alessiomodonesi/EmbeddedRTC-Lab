% --- SYSTEM PARAMETERS ---
% Insert the values estimated from a previous iteration
% the following parameters has been chosen from the 63.2 method estimation
% of motor 1 at Va = 3.84V
Kp = 1.6196;
Ki = 8.9979;
tau = 0.06;
mu = 2.4080; % Motor static gain
T = 0.26;  % Motor time constant
Ts = 0.01; % Sample time (10 ms based on STM32 config)

% --- SIMULATION SETUP ---
total_time = 5; % Simulate for 5 seconds
N = total_time / Ts;
t = (0:N-1) * Ts;

w_target = 10; % Desired target velocity (10 rad/s)

% Initialize memory arrays
w_measured = zeros(1, N);
applied_voltage = zeros(1, N);
error_array = zeros(1, N);

% Static variable for the integral
I = 0;

% Pre-padded array, in this way we implement the delay exp(-tau * s)
n_delay = round(tau/Ts);
Va_hist = zeros(1, N + n_delay); % zeros at the start: motor at rest

% --- CLOSED-LOOP SIMULATION CYCLE ---
for k = 1:N-1

    % Acquire current velocity
    w_current = w_measured(k);

    % Calculate error (Target - Measured)
    error = w_target - w_current;
    error_array(k) = error;

    % PI CONTROLLER (Translation of the C code)
    P = Kp * error;             % Proportional term
    I = I + Ki * error * Ts;    % Integral term with memory
    Va = P + I;                 % Voltage calculated by the controller

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

    % Anti-windup / Saturation: Motor cannot exceed supply voltage
    V_max = 6.0; % V_supply = 50% of V_PSU
    if Va > V_max
        Va = V_max;
    elseif Va < -V_max
        Va = -V_max;
    end

    applied_voltage(k) = Va;
    Va_hist(k + n_delay) = Va; % store with an offset

    % PHYSICAL MOTOR MODEL (Plant)
    % Calculate how the motor reacts to the applied voltage Va.
    % Discrete formula (Forward Euler) based on G(s) = mu / (1 + T*s)
    w_measured(k+1) = w_measured(k) + (Ts / T) * (-w_measured(k) + mu * Va_hist(k));

end

% --- PLOTS ---
figure;
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