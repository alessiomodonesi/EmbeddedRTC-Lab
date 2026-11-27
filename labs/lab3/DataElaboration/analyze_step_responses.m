%% Closed-loop step response analysis (interactive)
% Run this script in MATLAB from the folder containing the .mat files.

% Definitions
%   delay   : time from start of the recording to 10% of target
%   rise    : time from 10% to 90% of target
%   settling: last time the speed leaves +/-6% of target (at least +/-0.3 rad/s)
%   osc.    : dominant period and amplitude of the ripple after settling

files = dir('*.mat');
assert(~isempty(files), 'No .mat files found in %s', pwd);

fprintf('Closed-loop step response analysis\n');
fprintf('delay: recording start -> 10%% | rise: 10%% -> 90%% | settling: +/-6%% (min +/-0.3 rad/s)\n');

for k = 1:numel(files)
    name = files(k).name;
    S = load(name);
    assert(isfield(S, 'data'), '%s has no data struct', name);
    d = S.data;
    assert(numel(d.signals) >= 2, '%s: expected speed and voltage signals', name);
    t = d.time(:)';
    w = d.signals{1};
    assert(size(w, 2) == numel(t), '%s: signal length does not match time vector', name);

    tok = regexp(name, '([+-]?\d+(?:\.\d+)?)rads', 'tokens', 'once');
    assert(~isempty(tok), 'Cannot parse target speed from %s', name);
    target = str2double(tok{1});

    % The -6 file actually contains a positive run; trust the measured data.
    [~, imax] = max(abs(w(1, :)));
    resp_sign = sign(w(1, imax));
    if resp_sign == 0, resp_sign = sign(target); end
    warning_msg = '';
    if resp_sign ~= sign(target)
        warning_msg = sprintf('WARNING: data moves %+g rad/s, opposite to the file name. ', ...
                              resp_sign * abs(target));
        target = resp_sign * abs(target);
    end

    m1 = analyze_motor(t, w(1, :), target);
    m2 = analyze_motor(t, w(2, :), target);

    fprintf('\n=== %s ===\n', name);
    if ~isempty(warning_msg), fprintf('%s\n', warning_msg); end
    fprintf('Target: %g rad/s\n', target);
    fprintf('%-26s %16s %16s\n', 'Metric', 'Motor 1', 'Motor 2');
    fprintf('%-26s %16s %16s\n', 'Delay (to 10%)',       fmt_s(m1.delay),       fmt_s(m2.delay));
    fprintf('%-26s %16s %16s\n', 'Rise time (10-90%)',   fmt_s(m1.rise),        fmt_s(m2.rise));
    fprintf('%-26s %16s %16s\n', 'Settling time',        fmt_s(m1.settle),      fmt_s(m2.settle));
    fprintf('%-26s %16.2f %16.2f\n', 'Peak speed (rad/s)', m1.peak,             m2.peak);
    fprintf('%-26s %16s %16s\n', 'Overshoot',            fmt_pct(m1.overshoot), fmt_pct(m2.overshoot));
    fprintf('%-26s %16.2f %16.2f\n', 'Steady state (rad/s)', m1.ss,             m2.ss);
    fprintf('%-26s %16s %16s\n', 'Steady-state error',   fmt_pct(m1.sse),       fmt_pct(m2.sse));
    fprintf('%-26s %16s %16s\n', 'Oscillation period',   fmt_s(m1.osc_period),  fmt_s(m2.osc_period));
    fprintf('%-26s %16s %16s\n', 'Ripple amplitude',     fmt_amp(m1.osc_amp),   fmt_amp(m2.osc_amp));
    fprintf('%-26s %16s %16s\n', 'Settled interval',     fmt_iv(m1),            fmt_iv(m2));

    plot_response(name, t, w, target, m1, m2);
end

fprintf('\nAnalysis complete: %d file(s). Figures left open in the MATLAB window.\n', numel(files));

%% ------------------------------------------------------------------------
function m = analyze_motor(t, w, target)
m = struct('t0', NaN, 't10', NaN, 't90', NaN, 'y10', NaN, 'y90', NaN, ...
           'delay', NaN, 'rise', NaN, 'peak', NaN, 'peak_time', NaN, ...
           'peak_abs_time', NaN, 'overshoot', NaN, 'settle', NaN, ...
           'settle_abs', NaN, 'y_settle', NaN, 'ss', NaN, 'sse', NaN, ...
           'osc_period', NaN, 'osc_amp', NaN, 'settled_start', NaN, ...
           'settled_end', NaN);

at = abs(target);
s = sign(target);
if at == 0, return; end

ion = find(abs(w) >= max(0.01 * at, 0.02), 1, 'first');
iend = find(abs(w) >= 0.99 * at, 1, 'last');
if isempty(iend) % target never reached: keep the usable part of the run
    iend = find(abs(w) >= 0.5 * at, 1, 'last');
end
if isempty(ion) || isempty(iend) || iend <= ion, return; end

tt = t(ion:iend) - t(ion);
ww = s * w(ion:iend); % orient reverse runs as positive
m.t0 = t(ion);
m.settled_end = t(iend);

i10 = find(ww >= 0.1 * at, 1, 'first');
i90 = find(ww >= 0.9 * at, 1, 'first');
if ~isempty(i10)
    m.delay = t(ion + i10 - 1) - t(1); % delay from start of recording
    m.t10 = t(ion + i10 - 1);
    m.y10 = s * ww(i10);
end
if ~isempty(i90)
    m.t90 = t(ion + i90 - 1);
    m.y90 = s * ww(i90);
end
if ~isempty(i10) && ~isempty(i90) && i90 > i10
    m.rise = tt(i90) - tt(i10);
end

[peak, ip] = max(ww);
m.peak = s * peak;
m.peak_time = tt(ip);
m.peak_abs_time = t(ion + ip - 1);
m.overshoot = max(0, (peak - at) / at * 100);

n = numel(ww);
iss = max(1, n - max(10, round(0.1 * n)) + 1);
m.ss = s * mean(ww(iss:end));
m.sse = (m.ss - target) / at * 100;

tol = max(0.06 * at, 0.3);
outside = find(abs(ww - at) > tol, 1, 'last');
if isempty(outside)
    if isempty(i10), iset = 1; else, iset = i10; end
elseif outside < n
    iset = outside + 1;
else
    iset = [];
end

if ~isempty(iset)
    m.settle = tt(iset);
    m.settle_abs = t(ion + iset - 1);
    m.y_settle = s * ww(iset);
    m.settled_start = m.settle_abs;

    seg = ww(iset:end);
    tseg = tt(iset:end);
    N = numel(seg);
    if N >= 10
        m.osc_amp = (max(seg) - min(seg)) / 2;
        if m.osc_amp >= max(0.01 * at, 0.02)
            % Period from hysteresis zero crossings of the lightly smoothed
            % settled ripple; ignores fast quantisation noise.
            dt = median(diff(tseg));
            nwin = max(3, round(0.05 / dt));
            xs = movmean(seg, nwin) - mean(seg);
            thr = 0.3 * m.osc_amp;
            state = 0;
            crossings = [];
            for i = 2:N
                if state <= 0 && xs(i) >= thr
                    j = find(xs(1:i-1) <= 0, 1, 'last');
                    if ~isempty(j)
                        crossings(end+1) = interp1([xs(j), xs(i)], ...
                                                   [tseg(j), tseg(i)], 0); %#ok<AGROW>
                    end
                    state = 1;
                elseif state >= 0 && xs(i) <= -thr
                    state = -1;
                end
            end
            if numel(crossings) >= 3
                d = diff(crossings);
                d = d(d >= 0.1); % reject noise-scale crossings
                if ~isempty(d)
                    m.osc_period = median(d);
                end
            end
        end
    end
end
end

function plot_response(name, t, w, target, m1, m2)
figure('Name', name, 'NumberTitle', 'off', 'Color', 'w');
ax = axes;
hold(ax, 'on'); grid(ax, 'on');

c1 = [0.000 0.447 0.741];
c2 = [0.850 0.325 0.098];

h1 = plot(t, w(1, :), '-', 'Color', c1, 'LineWidth', 1.1);
h2 = plot(t, w(2, :), '-', 'Color', c2, 'LineWidth', 1.1);
ht = yline(target, 'k--', 'Target', 'LineWidth', 1.0, ...
           'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'on');

h10 = plot(nan, nan, 'o', 'Color', [0.3 0.3 0.3], 'MarkerSize', 7);
h90 = plot(nan, nan, 's', 'Color', [0.3 0.3 0.3], 'MarkerSize', 7);
hpk = plot(nan, nan, 'd', 'Color', [0.3 0.3 0.3], 'MarkerSize', 7);
hst = plot(nan, nan, '^', 'Color', [0.3 0.3 0.3], 'MarkerSize', 7);

highlight_motor(ax, m1, c1);
highlight_motor(ax, m2, c2);

xlabel('Time (s)');
ylabel('Speed (rad/s)');
title(sprintf('%s  |  target = %g rad/s', name, target), 'Interpreter', 'none');
xlim([t(1), t(end)]);

legend([h1, h2, ht, h10, h90, hpk, hst], ...
       {'Motor 1', 'Motor 2', 'Target', '10%', '90%', 'Peak', 'Settle'}, ...
       'Location', 'best');

txt = {sprintf('M1  td=%s  tr=%s  ts=%s  OS=%s  Tosc=%s', ...
               fmt_s(m1.delay), fmt_s(m1.rise), fmt_s(m1.settle), ...
               fmt_pct(m1.overshoot), fmt_s(m1.osc_period)), ...
       sprintf('M2  td=%s  tr=%s  ts=%s  OS=%s  Tosc=%s', ...
               fmt_s(m2.delay), fmt_s(m2.rise), fmt_s(m2.settle), ...
               fmt_pct(m2.overshoot), fmt_s(m2.osc_period))};
text(0.01, 0.99, txt, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
     'FontName', 'Consolas', 'FontSize', 8, 'BackgroundColor', 'w', ...
     'EdgeColor', [0.6 0.6 0.6], 'Margin', 4);
end

function highlight_motor(ax, m, col)
if isnan(m.t0), return; end

if ~isnan(m.settled_start)
    yl = ylim(ax);
    hp = patch(ax, [m.settled_start, m.settled_end, m.settled_end, m.settled_start], ...
               [yl(1), yl(1), yl(2), yl(2)], col, 'FaceAlpha', 0.07, ...
               'EdgeColor', 'none', 'HandleVisibility', 'off');
    uistack(hp, 'bottom');
    xline(ax, m.settle_abs, ':', 'Color', col, 'LineWidth', 1.2, ...
          'HandleVisibility', 'off');
end

pts = {m.t10, m.y10, 'o';
       m.t90, m.y90, 's';
       m.peak_abs_time, m.peak, 'd';
       m.settle_abs, m.y_settle, '^'};

for i = 1:size(pts, 1)
    if ~isnan(pts{i, 1}) && ~isnan(pts{i, 2})
        plot(ax, pts{i, 1}, pts{i, 2}, pts{i, 3}, 'Color', col, ...
             'MarkerSize', 7, 'LineWidth', 1.2, 'MarkerFaceColor', 'w', ...
             'HandleVisibility', 'off');
    end
end
end

function s = fmt_s(v)
if isnan(v), s = 'n/a'; else, s = sprintf('%.2f s', v); end
end

function s = fmt_pct(v)
if isnan(v), s = 'n/a'; else, s = sprintf('%.1f %%', v); end
end

function s = fmt_amp(v)
if isnan(v), s = 'n/a'; else, s = sprintf('%.2f rad/s', v); end
end

function s = fmt_iv(m)
if isnan(m.settled_start)
    s = 'n/a';
else
    s = sprintf('%.1f-%.1f s', m.settled_start, m.settled_end);
end
end
