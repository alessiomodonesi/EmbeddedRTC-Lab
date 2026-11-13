% LOAD DATA FROM THE .MAT FILE
% Load the u1 and w1_filt variables into the workspace directly from the file
load('motor_final_filtered.mat');

% Rename w1_filt to w1 to keep the script consistent
w1 = w1_filt;

% DETECT RISING AND FALLING EDGES OF THE VOLTAGE
% Create a logical vector: 1 when the motor is powered, 0 otherwise.
% Use > 0.1 instead of > 0 to prevent small sensor noise from being mistaken for steps
motor_on = u1 > 0.1; 

% diff() computes the variation. 
% 0 -> 1 gives +1 (start of the step)
% 1 -> 0 gives -1 (end of the step)
voltage_changes = diff(motor_on);

% Find the exact indices where each step starts and ends
% Add +1 to the start indices because diff() shifts the results by one position
start_indices = find(voltage_changes == 1) + 1; 
end_indices = find(voltage_changes == -1);

% Quick check: ensure we have the same number of start and end points
num_iterations = min(length(start_indices), length(end_indices));
fprintf('Found %d valid iterations.\n\n', num_iterations);

% Initialize two vectors to store the results of all trials
f_inf_array = zeros(1, num_iterations);
mu_array = zeros(1, num_iterations);

num_samples = 50; % Number of final samples used to calculate the average for f_inf

% FOR LOOP TO ANALYZE EACH ITERATION AUTOMATICALLY
for i = 1:num_iterations

    % Extract the start and end indices for the current iteration
    start_idx = start_indices(i);
    end_idx = end_indices(i);

    % Isolate the angular speed and voltage data for this specific step
    w_step = w1(start_idx:end_idx);
    u_step = u1(start_idx:end_idx);

    % Calculate the average armature voltage applied during this step (Va)
    Va = mean(u_step); 

    % 3. CALCULATE f_inf AND mu
    % Take the last num_samples of the angular velocity to compute the steady-state value
    steady_state_segment = w_step(end-num_samples+1 : end);
    f_inf = mean(steady_state_segment);

    % Calculate the static gain mu = f_inf / Va
    mu = f_inf / Va;

    % Store the results in the arrays created earlier
    f_inf_array(i) = f_inf;
    mu_array(i) = mu;

    % Print the results of the current iteration to the command window
    fprintf('Iteration %d:\n', i);
    fprintf('  Applied voltage (Va): %.2f V\n', Va);
    fprintf('  Steady-state speed (f_inf): %.2f rad/s\n', f_inf);
    fprintf('  Static gain (mu): %.2f (rad/s)/V\n\n', mu);

end