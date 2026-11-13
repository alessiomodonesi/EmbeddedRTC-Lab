% LOAD DATA
load('motor_final_filtered.mat');
w1 = w1_filt; 

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
tau_tan_array = zeros(1, num_iterations);
T_tan_array = zeros(1, num_iterations);

t_s = 0.01; % Sample time
num_samples = 50; 

% Create a figure to plot the tangents and visually check the results
figure;
hold on;

% 2. FOR LOOP TO ANALYZE THE ITERATIONS (TANGENT METHOD)
for i = 1:num_iterations

    start_idx = start_indices(i);
    end_idx = end_indices(i);
    w_step = w1(start_idx:end_idx);
    u_step = u1(start_idx:end_idx);

    Va = mean(u_step); 

    % Time vector for this specific step (starts at 0)
    t_step = (0:length(w_step)-1)' * t_s;

    % CALCULATE f_inf AND mu
    steady_state_segment = w_step(end-num_samples+1 : end);
    f_inf = mean(steady_state_segment);
    mu = f_inf / Va;

    % 3. TANGENT METHOD
    % Calculate the numerical derivative (dy/dt)
    dw = diff(w_step) / t_s;

    % Find the point of maximum slope (inflection point)
    [dw_max, idx_m] = max(dw);

    % Coordinates of the point of maximum slope
    % (Use idx_m because diff reduces the vector length by 1)
    t_m = t_step(idx_m);
    w_m = w_step(idx_m);

    % Calculate the intersections t0 and t1
    t0 = t_m - (w_m / dw_max);
    t1 = t_m + ((f_inf - w_m) / dw_max);

    % Estimate parameters tau and T
    tau = t0;
    T = t1 - t0;

    % Store in vectors
    f_inf_array(i) = f_inf;
    mu_array(i) = mu;
    tau_tan_array(i) = tau;
    T_tan_array(i) = T;

    % Print results to the command window
    %fprintf('Iteration %d (Tangent Method):\n', i);
    %fprintf('  Speed (f_inf): %.2f rad/s\n', f_inf);
    %fprintf('  Time delay (tau): %.4f s\n', tau);
    %fprintf('  Time constant (T):  %.4f s\n\n', T);

    % --- VERIFICATION PLOT ---
    % Select a subplot for each iteration (up to 5, or 6 if you have more)
    subplot(ceil(num_iterations/2), 2, i);
    plot(t_step, w_step, 'b', 'LineWidth', 1.5); hold on;

    % Tangent line equation: y = m*(t - t_m) + y_m
    y_tan = dw_max * (t_step - t_m) + w_m;

    % Plot the tangent line
    plot(t_step, y_tan, 'g--', 'LineWidth', 1.5);

    % Plot the steady-state line
    yline(f_inf, 'r--', 'LineWidth', 1);

    % Set axis limits to clearly view the intersection
    xlim([0, t1 + 2*T]);
    ylim([0, f_inf * 1.2]);
    title(sprintf('Iteration %d', i));
    xlabel('Time [s]'); ylabel('Speed');
    grid on;
    % ----------------------------------------------------
end

% --- CALCULATE PI PARAMETERS (Tangent Method) ---
Kp_tan = (0.9 ./ mu_array) .* (T_tan_array ./ tau_tan_array);
Ti_tan = 3 .* tau_tan_array;
Ki_tan = Kp_tan ./ Ti_tan;

% Display the results in the command window
disp('--- PI Parameters (Tangent Method) ---');
for i = 1:num_iterations
    fprintf('Iteration %d: Kp = %.4f, Ki = %.4f\n', i, Kp_tan(i), Ki_tan(i));
end