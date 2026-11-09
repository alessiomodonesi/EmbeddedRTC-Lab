% MATLAB script: DC motor analysis and PI tuning (TurtleBot)
% Requirements: Control System Toolbox, Signal Processing Toolbox
%
% WHAT TO CHANGE (real data):
%   - Ts, V_supply, duty_step, fc
%   - USE_MOCK = false
%   - the arrays t, duty and speed with the data acquired by telemetry
%
% WHAT IS MOCK DATA (example only, do not change):
%   - the whole "MOCK DATA" block: simulated response and noise
%
% Method (Embedded08 lecture): two synchronised traces are acquired, duty
% cycle and speed. The time of the step is taken from the duty trace, NOT
% from the origin of the MATLAB time axis (the PC and the TurtleBot have
% asynchronous clocks).

clear variables; close all; clc;

%% 1. ACQUIRED DATA - CHANGE THESE VALUES
Ts = 0.01;                  % Sampling period [s] (10 ms, Timer 6)
V_supply = 9.96;            % Bench supply voltage [V]
duty_step = 0.64;           % Duty of the step (64% = weighted average, lecture)
fc = 2;                     % Low-pass cutoff frequency [Hz]; 0 = no filter

% Real data: paste the acquired arrays here (same length, same order and
% synchronised in the same telemetry packet). They are used only if
% USE_MOCK = false.
USE_MOCK = true;
t = [];
duty = [];
speed = [];

%% 2. MOCK DATA - EXAMPLE ONLY, DO NOT CHANGE
% This block only lets you test the script without the robot.
if USE_MOCK
    t = 0:Ts:3;

    % Step applied at t_step, not at t = 0 (as in real life)
    t_step = 0.5;
    duty = zeros(size(t));
    duty(t >= t_step) = duty_step;

    % "True" parameters used to generate the simulated response
    tau_true = 0.15;
    T_true = 0.25;
    mu_true = 2.5;
    Va_step = duty_step * V_supply;

    speed = zeros(size(t));
    idx_start = find(t >= t_step + tau_true, 1);
    speed(idx_start:end) = mu_true * Va_step * ...
        (1 - exp(-(t(idx_start:end) - t_step - tau_true) / T_true));

    % Simulated encoder noise: this is for the mock only. With real data it
    % is not needed, because the noise is already present in the measurement.
    noise_std = 0.05;
    speed = speed + noise_std * randn(size(speed));
end

% Real instant when the step is applied, taken from the duty trace
idx_step = find(duty >= 0.5 * max(duty), 1);
t_step = t(idx_step);

%% 3. LOW-PASS FILTER (OPTIONAL)
% fc = 0 turns the filter off. If the data is already clean, you do not need it.
% Teacher's filter (filter_example.m): continuous first-order filter
% H(s) = 2*pi*fc / (s + 2*pi*fc), discretised with Tustin and applied with
% filter(). Because it is causal, it adds a delay: choose fc by trial and
% error, high enough to remove noise but not so high that it changes T and tau.
if fc > 0
    sysF = tf(2*pi*fc, [1, 2*pi*fc]);
    sysF_d = c2d(sysF, Ts, 'tustin');
    [numF, denF] = tfdata(sysF_d, 'v');
    speed_filtered = filter(numF, denF, speed);
else
    speed_filtered = speed;
    disp('Filtro disattivato (fc = 0). Il metodo della tangente puo'' essere rumoroso.');
end

% Actual step voltage
Va_step = duty_step * V_supply;

%% 4. IDENTIFICATION: 63.2% METHOD (mandatory)
% Steady-state speed (mean of the last 50 samples)
f_inf = mean(speed_filtered(end-50:end));
mu = f_inf / Va_step;

% Pure delay: time between the duty step and the start of the rise.
% Threshold at 5% of f_inf to avoid counting the noise.
threshold_start = 0.05 * f_inf;
idx_tau = find(speed_filtered >= threshold_start, 1);
tau = t(idx_tau) - t_step;

% Time constant: 63.2% is reached after tau + T from the step
threshold_63 = 0.632 * f_inf;
idx_T = find(speed_filtered >= threshold_63, 1);
T = t(idx_T) - t_step - tau;

assert(all(isfinite([mu tau T])) && mu > 0 && T > 0, ...
    'Identificazione 63.2%% non valida: controllare dati, filtro e soglie.');

fprintf('--- Identificazione 63.2%% ---\n');
fprintf('Guadagno statico (mu): %.4f (rad/s)/V\n', mu);
fprintf('Ritardo puro (tau):    %.4f s\n', tau);
fprintf('Costante di tempo (T): %.4f s\n\n', T);

%% 5. IDENTIFICATION: TANGENT METHOD (mandatory)
% Numerical derivative of the (possibly filtered) signal, only after the step
dy = gradient(speed_filtered, t);
idx_rise = find(t >= t_step);
[slope_max, i_m] = max(dy(idx_rise));
idx_m = idx_rise(i_m);
t_m = t(idx_m);
y_m = speed_filtered(idx_m);

if slope_max <= 0
    error('Pendenza massima non positiva: controllare i dati o il filtro.');
end

% Intersections of the tangent with the time axis and with f_inf
t0 = t_m - y_m / slope_max;
t1 = t_m + (f_inf - y_m) / slope_max;
tau_tan = max(t0 - t_step, 0);
T_tan = t1 - t0;

assert(isfinite(tau_tan) && isfinite(T_tan) && T_tan > 0, ...
    'Metodo della tangente non valido: controllare filtro e dati.');

fprintf('--- Identificazione Tangente ---\n');
fprintf('Ritardo puro (tau):    %.4f s\n', tau_tan);
fprintf('Costante di tempo (T): %.4f s\n\n', T_tan);

%% 6. MOTOR TRANSFER FUNCTION (FOPTD)
s = tf('s');
G_base = mu / (1 + T*s);    % First-order model without delay
G_motor = G_base;
G_motor.InputDelay = tau;

disp('Funzione di trasferimento approssimata del motore (63.2%):');
G_motor

%% 7. PI TUNING (Ziegler-Nichols, open loop)
% Valid only if tau > 0. Alternative when tau = 0: closed loop
% (Kp = 0.45*Kcr, Ti = Tcr/1.2).
if tau > 0
    Kp = (0.9 / mu) * (T / tau);
    Ti = 3 * tau;
    Ki = Kp / Ti;
else
    warning('Ritardo nullo: Ziegler-Nichols anello aperto non applicabile. Uso valori di default.');
    Kp = 1.0;
    Ti = 0.5;
    Ki = Kp / Ti;
end

C_PI = Kp + Ki/s;

fprintf('--- Parametri Controllore PI ---\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n\n', Ki);

%% 8. PLOTTING AND ROOT LOCUS
figure('Name', 'Analisi Motore TurtleBot', 'NumberTitle', 'off', ...
    'Position', [100 100 1000 600]);

% Step response with both methods
subplot(2,2,1);
h_raw = plot(t, speed, 'Color', [0.7 0.7 0.7]); hold on;
h_filtered = plot(t, speed_filtered, 'b', 'LineWidth', 1.5);
yline(f_inf, 'r--', 'Regime (f_{\infty})', 'LabelHorizontalAlignment', 'left');
yline(threshold_63, 'g--', '63.2% f_{\infty}', 'LabelHorizontalAlignment', 'left');
plot(t(idx_T), speed_filtered(idx_T), 'ko', 'MarkerFaceColor', 'g');
h_tan = plot(t, y_m + slope_max*(t - t_m), 'm-.', 'LineWidth', 1.2);
xline(t_step, 'k:', 'Scalino duty');
title('Risposta allo Scalino (63.2% e Tangente)');
xlabel('Tempo [s]'); ylabel('Velocità [rad/s]');
legend([h_raw h_filtered h_tan], {'Dati Raw', 'Dati Filtrati', 'Tangente'}, ...
    'Location', 'best');
grid on;

% Model vs data comparison
subplot(2,2,2);
[y_sim, t_sim] = step(Va_step * G_motor, t);
plot(t, speed_filtered, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, y_sim, 'r--', 'LineWidth', 1.5);
title('Confronto Modello Identificato vs Dati Sperimentali');
xlabel('Tempo [s]'); ylabel('Velocità [rad/s]');
legend('Dati Filtrati', 'Simulazione Modello FOPTD', 'Location', 'best');
grid on;

% Root locus (delay approximated with Pade)
subplot(2,2,[3 4]);
[num_pade, den_pade] = pade(tau, 1);
G_pade = tf(num_pade, den_pade) * G_base;
C_structure = (s + 1/Ti) / s;
L_openloop = C_structure * G_pade;

rlocus(L_openloop);
title(sprintf('Luogo delle Radici (PI con Ti = %.3f s)', Ti));
grid on;
