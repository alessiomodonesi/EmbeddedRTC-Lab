close all; clear; clc;

Ts = 0.01;      % sample time [s]
fc = 2.0;       % low-pass cut-off [Hz], same as analyze_motor_data.m
fig_dir = fullfile('..', '..', '..', '..', 'reports', 'lab2', 'figures');

% same filter as analyze_motor_data.m: H(s) = 2*pi*fc/(s + 2*pi*fc), Tustin
sysF_d = c2d(tf(2*pi*fc, [1, 2*pi*fc]), Ts, 'tustin');
[numF, denF] = tfdata(sysF_d, 'v');

files  = {'motor_data_forward.mat', 'motor_data_reverse.mat', ...
          'motor_data_crossed.mat', 'motor_data_coast_forward.mat'};
labels = {'brake forward', 'brake reverse', 'crossed', 'coast forward'};

%% 1. IDENTIFICATION OF ALL DATA SETS 
res = struct();   % res(f, m).p632 / .ptan
for f = 1:numel(files)
    [w, u] = load_motor_data(files{f});
    for m = 1:2
        wf = filter(numF, denF, w(m,:));
        res(f, m).p632 = method632(wf, u(m,:), Ts);
        res(f, m).ptan = methodTangent(wf, u(m,:), Ts);

        fprintf('\n=== %s, motor %d ===\n', labels{f}, m);
        fprintf(' Va[V]  f_inf  mu      T63   tau63 | Ttan   tautan\n');
        for i = 1:numel(res(f, m).p632)
            a = res(f, m).p632(i); b = res(f, m).ptan(i);
            fprintf(' %4.2f  %5.2f  %5.3f   %4.2f  %5.3f | %5.3f  %5.3f\n', ...
                a.Va, a.finf, a.mu, a.T, a.tau, b.T, b.tau);
        end
        fprintf(' mean            %5.3f   %5.3f %5.3f | %5.3f  %5.3f\n', ...
            mean([res(f,m).p632.mu]), mean([res(f,m).p632.T]), mean([res(f,m).p632.tau]), ...
            mean([res(f,m).ptan.T]), mean([res(f,m).ptan.tau]));
        fprintf(' median          %5.3f   %5.3f %5.3f | %5.3f  %5.3f\n', ...
            median([res(f,m).p632.mu]), median([res(f,m).p632.T]), median([res(f,m).p632.tau]), ...
            median([res(f,m).ptan.T]), median([res(f,m).ptan.tau]));
    end
end

% reverse / forward gain ratio, step by step
fprintf('\n=== Gain ratio reverse / forward ===\n');
for m = 1:2
    fprintf(' motor %d: %s\n', m, sprintf('%.2f ', ...
        [res(2,m).p632.mu] ./ [res(1,m).p632.mu]));
end

%% 2. FILTER DELAY: 3.84 V STEP (step number 3) ANALYSED IN FIVE WAYS
[w, u] = load_motor_data('motor_data_forward.mat');
k = 3;   % 3.84 V step
fprintf('\n=== 3.84 V step, with and without the filter delay ===\n');
fprintf(' method                      mu      T      tau\n');
lsfit = cell(1, 2);
for m = 1:2
    w_c  = filter(numF, denF, w(m,:));     % causal (used in the report)
    w_zp = filtfilt(numF, denF, w(m,:));   % zero-phase
    w_r  = w(m,:);                         % raw

    a  = method632(w_c,  u(m,:), Ts);
    b  = methodTangent(w_c, u(m,:), Ts);
    c  = method632(w_zp, u(m,:), Ts);
    r  = method632(w_r,  u(m,:), Ts);
    lsfit{m} = fitFOPDT(w_r, u(m,:), Ts);
    L  = lsfit{m};

    fprintf(' motor %d\n', m);
    fprintf('  63.2%%, causal LPF          %5.3f  %5.2f  %5.2f\n', a(k).mu, a(k).T, a(k).tau);
    fprintf('  tangent, causal LPF        %5.3f  %5.3f  %5.3f\n', b(k).mu, b(k).T, b(k).tau);
    fprintf('  63.2%%, zero-phase LPF      %5.3f  %5.2f  %5.2f\n', c(k).mu, c(k).T, c(k).tau);
    fprintf('  63.2%%, raw signal          %5.3f  %5.2f  %5.2f\n', r(k).mu, r(k).T, r(k).tau);
    fprintf('  least squares, raw signal  %5.3f  %5.3f  %5.3f\n', ...
        L(k).mu, L(k).T, L(k).tau);
end

%% 3. OSCILLATION ON THE PLATEAUS vs WHEEL ROTATION FREQUENCY
fprintf('\n=== Oscillation on the plateaus (brake forward) ===\n');
nfft = 4096;
for m = 1:2
    [s_idx, e_idx] = find_steps(u(m,:));
    for i = 1:numel(s_idx)
        x  = w(m, s_idx(i)+100 : e_idx(i));    % skip the first second
        wm = mean(x);
        x  = x - wm;
        n  = numel(x);
        win = 0.5 - 0.5*cos(2*pi*(0:n-1)/(n-1));   % Hann window
        X  = abs(fft(x .* win, nfft));
        X  = X(1:nfft/2+1);
        fr = (0:nfft/2) / (nfft*Ts);
        valid = find(fr > 0.3);
        [~, j] = max(X(valid));
        fprintf(' motor %d, step %d: w = %5.2f rad/s, oscillation %.2f Hz, wheel rotation %.2f Hz\n', ...
            m, i, wm, fr(valid(j)), wm/(2*pi));
    end
end

%% 4. ZIEGLER-NICHOLS GAINS, BOTH MOTORS (design model)
% design model: 63.2 % method, causal filter, 3.84 V step
design = [2.408 0.26 0.06;    % motor 1: mu, T, tau
          1.916 0.25 0.06];   % motor 2
fprintf('\n=== Ziegler-Nichols ===\n');
for m = 1:2
    mu = design(m,1); T = design(m,2); tau = design(m,3);
    Kp1 = 0.9*T/(mu*tau); Ti1 = 3*tau; Ki1 = Kp1/Ti1;
    wc  = fzero(@(x) -x*tau - atan(x*T) + pi, [1e-3, 1e4]);
    Kcr = sqrt(1 + (wc*T)^2)/mu;  Tcr = 2*pi/wc;
    Kp2 = 0.45*Kcr; Ti2 = Tcr/1.2; Ki2 = Kp2/Ti2;
    fprintf(' motor %d open loop : Kp = %.4f  Ti = %.3f  Ki = %.4f\n', m, Kp1, Ti1, Ki1);
    fprintf(' motor %d closed loop: wc = %.2f  Tcr = %.4f  Kcr = %.3f  Kp = %.4f  Ti = %.4f  Ki = %.4f\n', ...
        m, wc, Tcr, Kcr, Kp2, Ti2, Ki2);
end

%% 5. CLOSED-LOOP ANALYSIS (gains of the firmware, reference 10 rad/s)
gains = [1.6196  8.9979;     % KP1, KI1
         1.9574 10.8747];    % KP2, KI2
% fitted model, rounded as in the report (tau to whole samples)
fitted = [2.387 0.206 0.01;
          1.950 0.191 0.02];
w_ref = 10;
fprintf('\n=== Closed loop, step 0 -> %g rad/s ===\n', w_ref);
fprintf(' motor  plant / PI              PM[deg] GM[dB] wc[rad/s] | tr[s]  OS[%%]  ts[s]  sat[s]\n');
for m = 1:2
    Kp = gains(m,1); Ki = gains(m,2);
    cases = {'design', design(m,:), 'linear';
             'design', design(m,:), 'clamp';
             'design', design(m,:), 'aw';
             'fitted', fitted(m,:), 'aw'};
    for c = 1:size(cases, 1)
        pl = cases{c,2};
        [t, wsim, Vsim] = simPI(Kp, Ki, pl(1), pl(2), pl(3), w_ref, 5, cases{c,3}, Ts);
        M = stepMetrics(t, wsim, w_ref);
        sat = sum(abs(Vsim) >= 5.999) * Ts;
        if strcmp(cases{c,3}, 'linear') || strcmp(cases{c,1}, 'fitted')
            P = tf(pl(1), [pl(2) 1], 'InputDelay', pl(3));
            [Gm, Pm, ~, Wcp] = margin(pid(Kp, Ki) * P);
            GmdB = 20*log10(Gm);
            mstr = sprintf('%6.1f  %5.1f  %6.1f', Pm, GmdB, Wcp);
        else
            mstr = sprintf('%6s  %5s  %6s', '', '', '');
        end
        if strcmp(cases{c,3}, 'linear'), sstr = '   --'; else, sstr = sprintf('%5.2f', sat); end
        fprintf('   %d    %-6s %-14s  %s    | %5.2f  %5.1f  %5.2f  %s\n', ...
            m, cases{c,1}, cases{c,3}, mstr, M.tr, M.os, M.ts, sstr);
    end
end

%% 6. FIGURES
c1 = [0.12 0.37 0.75]; c2 = [0.84 0.15 0.16]; c3 = [0.17 0.63 0.17]; c4 = [0.5 0.5 0.5];

%filter delay on the 3.84 V step of motor 1
[w, u] = load_motor_data('motor_data_forward.mat');
[s_idx, ~] = find_steps(u(1,:));
s0 = s_idx(3);
idx = (s0-30):(s0+119);
t_rel = (idx - s0) * Ts;
L = lsfit{1}(3);
t_mod = (0:119) * Ts;
y_mod = fopdtResponse([L.mu L.T L.tau], numel(t_mod), L.Va, Ts);
w_c  = filter(numF, denF, w(1,:));
w_zp = filtfilt(numF, denF, w(1,:));

figA = figure('Color', 'w', 'Position', [100 100 640 360]);
theme(figA, 'light');   % white background even if MATLAB is in dark mode
stairs(t_rel, w(1,idx), 'Color', c4, 'LineWidth', 1); hold on;
plot(t_rel, w_c(idx),  'Color', c1, 'LineWidth', 1.8);
plot(t_rel, w_zp(idx), '--', 'Color', c3, 'LineWidth', 1.6);
plot(t_mod, y_mod, ':', 'Color', c2, 'LineWidth', 1.6);
xline(0, 'k');
xlim([t_rel(1) t_rel(end)]); grid on;
xlabel('time from voltage step [s]'); ylabel('\omega_1 [rad/s]');
title('Motor 1, step to 3.84 V');
legend('raw encoder speed', 'causal LPF 2 Hz (used in identification)', ...
       'zero-phase LPF 2 Hz (filtfilt)', 'least-squares FOPDT fit on raw data', ...
       'Location', 'southeast');
exportgraphics(figA, fullfile(fig_dir, 'filter_delay.png'), 'Resolution', 200);

%mu and T vs step amplitude
figB = figure('Color', 'w', 'Position', [100 100 760 330]);
theme(figB, 'light');   % white background even if MATLAB is in dark mode
sets = [1 2 4];                       % forward, reverse, coast
styles = {'-o', '--s', ':^'};
names  = {'brake fwd', 'brake rev', 'coast fwd'};
cols   = {c1, c2};
leg = {};
for q = 1:3
    for m = 1:2
        r = res(sets(q), m).p632;
        if q == 2, mfc = 'w'; else, mfc = cols{m}; end
        subplot(1,2,1); hold on;
        plot([r.Va], [r.mu], styles{q}, 'Color', cols{m}, 'MarkerFaceColor', mfc, ...
             'MarkerSize', 5, 'LineWidth', 1.3);
        subplot(1,2,2); hold on;
        plot([r.Va], [r.T], styles{q}, 'Color', cols{m}, 'MarkerFaceColor', mfc, ...
             'MarkerSize', 5, 'LineWidth', 1.3);
        leg{end+1} = sprintf('%s, M%d', names{q}, m);
    end
end
subplot(1,2,1); grid on; box on;
xlabel('step amplitude V_a [V]'); ylabel('\mu [rad/s/V]');
legend(leg, 'Location', 'northwest', 'FontSize', 7);
subplot(1,2,2); grid on; box on;
xlabel('step amplitude V_a [V]'); ylabel('T [s]');
exportgraphics(figB, fullfile(fig_dir, 'params_vs_voltage.png'), 'Resolution', 200);

%closed loop, both motors
figC = figure('Color', 'w', 'Position', [100 100 760 480]);
theme(figC, 'light');
for m = 1:2
    Kp = gains(m,1); Ki = gains(m,2);
    [t, w_sat, V_sat] = simPI(Kp, Ki, design(m,1), design(m,2), design(m,3), w_ref, 2.5, 'clamp', Ts);
    [~, w_aw,  V_aw ] = simPI(Kp, Ki, design(m,1), design(m,2), design(m,3), w_ref, 2.5, 'aw', Ts);
    [~, w_fit, V_fit] = simPI(Kp, Ki, fitted(m,1), fitted(m,2), fitted(m,3), w_ref, 2.5, 'aw', Ts);

    subplot(2,2,m); hold on; grid on; box on;
    plot(t, w_sat, 'Color', c2, 'LineWidth', 1.5);
    plot(t, w_aw,  'Color', c1, 'LineWidth', 1.5);
    plot(t, w_fit, '--', 'Color', c3, 'LineWidth', 1.4);
    yline(w_ref, 'k:');
    title(sprintf('Motor %d (K_p=%.4f, K_i=%.4f)', m, Kp, Ki), 'FontWeight', 'normal');
    if m == 1
        ylabel('\omega [rad/s]');
        legend('design model, saturation only', 'design model, anti-windup', ...
               'fitted model, anti-windup', 'Location', 'southeast', 'FontSize', 7);
    end

    subplot(2,2,m+2); hold on; grid on; box on;
    plot(t, V_sat, 'Color', c2, 'LineWidth', 1.5);
    plot(t, V_aw,  'Color', c1, 'LineWidth', 1.5);
    plot(t, V_fit, '--', 'Color', c3, 'LineWidth', 1.4);
    ylim([0 6.5]); xlabel('time [s]');
    if m == 1, ylabel('u [V]'); end
end
exportgraphics(figC, fullfile(fig_dir, 'closed_loop_both.png'), 'Resolution', 200);

fprintf('\nFigures saved in %s\n', fig_dir);


%%

function [w, u] = load_motor_data(fname)
% returns 2xN speed [rad/s] and 2xN voltage [V] from a data-logger file
S = load(fname);
w = double(S.data.signals{1});
u = double(S.data.signals{2});
end

function [s_idx, e_idx] = find_steps(u)
% same edge detection as method_632.m / method_tangent.m
motor_on = u > 0.1;
d = diff(motor_on);
s_idx = find(d == 1) + 1;
e_idx = find(d == -1);
n = min(numel(s_idx), numel(e_idx));
s_idx = s_idx(1:n); e_idx = e_idx(1:n);
end

function out = method632(w, u, Ts)
% 63.2 % method exactly as in method_632.m
[s_idx, e_idx] = find_steps(u);
out = struct('Va', {}, 'finf', {}, 'mu', {}, 'tau', {}, 'T', {});
for i = 1:numel(s_idx)
    ws = w(s_idx(i):e_idx(i));
    Va = mean(u(s_idx(i):e_idx(i)));
    finf = mean(ws(end-49:end));
    idx5  = find(ws >= 0.05*finf,  1, 'first');
    idx63 = find(ws >= 0.632*finf, 1, 'first');
    out(i).Va = Va; out(i).finf = finf; out(i).mu = finf/Va;
    out(i).tau = (idx5 - 1)*Ts;
    out(i).T   = (idx63 - idx5)*Ts;
end
end

function out = methodTangent(w, u, Ts)
% tangent method exactly as in method_tangent.m
[s_idx, e_idx] = find_steps(u);
out = struct('Va', {}, 'finf', {}, 'mu', {}, 'tau', {}, 'T', {});
for i = 1:numel(s_idx)
    ws = w(s_idx(i):e_idx(i));
    Va = mean(u(s_idx(i):e_idx(i)));
    finf = mean(ws(end-49:end));
    t  = (0:numel(ws)-1)*Ts;
    dw = diff(ws)/Ts;
    [dw_max, im] = max(dw);
    t0 = t(im) - ws(im)/dw_max;
    t1 = t(im) + (finf - ws(im))/dw_max;
    out(i).Va = Va; out(i).finf = finf; out(i).mu = finf/Va;
    out(i).tau = t0; out(i).T = t1 - t0;
end
end

function y = fopdtResponse(p, n, Va, Ts)
% step response of mu*exp(-tau*s)/(1+T*s) to a step of amplitude Va
mu = p(1); T = max(p(2), 1e-4); tau = p(3);
t = (0:n-1)*Ts;
y = zeros(1, n);
k = t > tau;
y(k) = mu*Va*(1 - exp(-(t(k) - tau)/T));
end

function out = fitFOPDT(w, u, Ts)
% least-squares fit of mu, T, tau on each raw step (fminsearch, no toolbox)
[s_idx, e_idx] = find_steps(u);
out = struct('Va', {}, 'mu', {}, 'T', {}, 'tau', {});
opts = optimset('TolX', 1e-7, 'TolFun', 1e-9, 'MaxFunEvals', 1e4, 'MaxIter', 1e4);
for i = 1:numel(s_idx)
    ws = w(s_idx(i):e_idx(i));
    Va = mean(u(s_idx(i):e_idx(i)));
    n  = numel(ws);
    % bounds as in the report: mu in [0,10], T in [0.01,2], tau in [0,0.3]
    clampP = @(p) [min(max(p(1),0),10), min(max(p(2),0.01),2), min(max(p(3),0),0.3)];
    cost = @(p) sum((fopdtResponse(clampP(p), n, Va, Ts) - ws).^2);
    p = clampP(fminsearch(cost, [2 0.2 0.02], opts));
    out(i).Va = Va; out(i).mu = p(1); out(i).T = p(2); out(i).tau = p(3);
end
end

function [t, w, V] = simPI(Kp, Ki, mu, T, tau, w_ref, t_end, mode, Ts)
% discrete closed loop as in PI_sim.m
% mode: 'linear' (no saturation), 'clamp' (saturation only),
%       'aw' (saturation + conditional integration)
Vmax = 6.0;
N  = round(t_end/Ts);
nd = round(tau/Ts);
w = zeros(1, N); V = zeros(1, N); hist = zeros(1, N + nd);
I = 0;
for k = 1:N-1
    e = w_ref - w(k);
    P = Kp*e;
    switch mode
        case 'linear'
            I = I + Ki*e*Ts;  Va = P + I;
        case 'clamp'
            I = I + Ki*e*Ts;  Va = min(Vmax, max(-Vmax, P + I));
        case 'aw'
            Vun  = P + I;
            Inew = I + Ki*e*Ts;
            if (Vun >= -Vmax && Vun <= Vmax) || (Vun > 0 && e < 0) || (Vun < 0 && e > 0)
                I = Inew;
            end
            Va = min(Vmax, max(-Vmax, P + I));
    end
    V(k) = Va;
    hist(k + nd) = Va;
    w(k+1) = w(k) + (Ts/T)*(-w(k) + mu*hist(k));
end
V(N) = V(N-1);
t = (0:N-1)*Ts;
end

function M = stepMetrics(t, w, w_ref)
% 10-90 % rise time, overshoot [%], 2 % settling time
i10 = find(w >= 0.1*w_ref, 1, 'first');
i90 = find(w >= 0.9*w_ref, 1, 'first');
M.tr = t(i90) - t(i10);
M.os = max(0, (max(w) - w_ref)/w_ref*100);
out = find(abs(w - w_ref) > 0.02*w_ref);
if isempty(out), M.ts = 0; else, M.ts = t(out(end) + 1); end
end
