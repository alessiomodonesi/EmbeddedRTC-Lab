% LOAD DATA FROM THE .MAT FILE
load('motor_final_filtered.mat');
w1 = w1_filt; % Rename for convenience

% DETECT RISING AND FALLING EDGES
motor_on = u1 > 0.1; 
voltage_changes = diff(motor_on);
start_indices = find(voltage_changes == 1) + 1; 
end_indices = find(voltage_changes == -1);

num_iterations = min(length(start_indices), length(end_indices));
fprintf('Found %d valid iterations.\n\n', num_iterations);

% Initialize vectors to store the results
f_inf_array = zeros(1, num_iterations);
mu_array = zeros(1, num_iterations);
tau_array = zeros(1, num_iterations);
T_array = zeros(1, num_iterations);

% Constant parameters
num_samples = 50; 
t_s = 0.01; % Sample time in seconds

% FOR LOOP TO ANALYZE THE ITERATIONS
for i = 1:num_iterations

    % Extract data for the current iteration
    start_idx = start_indices(i);
    end_idx = end_indices(i);
    w_step = w1(start_idx:end_idx);
    u_step = u1(start_idx:end_idx);

    Va = mean(u_step); 

    % CALCULATE f_inf AND mu
    steady_state_segment = w_step(end-num_samples+1 : end);
    f_inf = mean(steady_state_segment);
    mu = f_inf / Va;

    % 3. CALCULATE TIME DELAY (tau) AND TIME CONSTANT (T) AT 63.2%
    % Calculate the 5% and 63.2% speed thresholds
    threshold_5  = 0.05 * f_inf;
    threshold_63 = 0.632 * f_inf;

    % Find the first index where the speed exceeds the respective thresholds.
    % The find(..., 1, 'first') function returns only the first matching value.
    idx_5  = find(w_step >= threshold_5, 1, 'first');
    idx_63 = find(w_step >= threshold_63, 1, 'first');

    % Calculate tau: elapsed time from the start of the step (index 1) up to 5%
    % Subtract 1 because if the index is 1, the elapsed time is 0.
    tau = (idx_5 - 1) * t_s;

    % Calculate T: elapsed time between reaching 5% and 63.2%
    T = (idx_63 - idx_5) * t_s;

    % Store the results in the arrays
    f_inf_array(i) = f_inf;
    mu_array(i) = mu;
    tau_array(i) = tau;
    T_array(i) = T;

    % Print results to the command window
    %fprintf('Iteration %d:\n', i);
    %fprintf('  Voltage (Va): %.2f V\n', Va);
    %fprintf('  Speed (f_inf): %.2f rad/s\n', f_inf);
    %fprintf('  Gain (mu): %.4f (rad/s)/V\n', mu);
    %fprintf('  Time delay (tau): %.4f s\n', tau);
    %fprintf('  Time constant (T):  %.4f s\n\n', T);

end

% --- CALCULATE PI PARAMETERS (63.2% Method) ---
% Use "./" and ".*" operators for element-wise operations on arrays
Kp_63 = (0.9 ./ mu_array) .* (T_array ./ tau_array);
Ti_63 = 3 .* tau_array;
Ki_63 = Kp_63 ./ Ti_63;

% Display the results in the command window
disp('--- PI Parameters (63.2% Method) ---');
for i = 1:num_iterations
    fprintf('Iteration %d: Kp = %.4f, Ki = %.4f\n', i, Kp_63(i), Ki_63(i));
end