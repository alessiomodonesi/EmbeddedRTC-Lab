close all
clear
clc

%% === Filter parameters ===
fc = 1;        % Cutoff frequency [Hz]
Ts = 0.01;     % Sampling period [s]

%% === Define an example noisy signal ===
t = 0:Ts:2;                         % Time vector [s]
y_true = 1 - exp(-t/0.2);           % Original signal
noise = 0.02 * randn(size(y_true)); % Additive white noise
y = y_true + noise;                 % Noisy signal

%% === Define the continuous-time low-pass filter ===
% The continuous-time transfer function of a first-order LPF is:
%
%              (2*pi*fc)
%   H(s) = -----------------
%          (s + 2*pi*fc)
%
% This filter attenuates frequencies higher than fc [Hz].
% It is a standard first-order low-pass system, equivalent to an RC filter.
sysF = tf(2*pi*fc, [1, 2*pi*fc]);

%% === Discretize the filter (continuous -> discrete) ===
% The signal 'y' is discrete (sampled at Ts seconds), while 'sysF' is
% continuous-time (defined in the Laplace domain, variable s).
% To apply the filter digitally, we must convert H(s) → H(z).
%
% MATLAB's 'c2d()' function performs this conversion. We use the 'tustin'
% method, also known as the *bilinear transformation*, which replaces:
%
%       s = (2/Ts) * (1 - z^-1) / (1 + z^-1)
%
% The Tustin transform:
%   * Preserves the frequency response well at low frequencies
%   * Maps the stable left-half s-plane to the inside of the unit circle (stable in z)
%   * Avoids aliasing of poles and zeros
%   * Produces a stable, causal discrete-time filter suitable for implementation
%
% The resulting discrete filter H(z) ≈ H(s) in behavior for frequencies
% below Nyquist (Fs/2).
sysF_d = c2d(sysF, Ts, 'tustin');

% Extract numerator and denominator coefficients for use with 'filter'
[numF, denF] = tfdata(sysF_d, 'v');

%% === Apply the digital filter to the noisy signal ===
yf = filter(numF, denF, y);

%% === Plot the signals ===
figure('Color', 'w'); % White background
plot(t, y, 'Color', 'g', 'LineWidth', 1.8); hold on;
plot(t, yf, 'b', 'LineWidth', 1.8);
plot(t, y_true, 'r--', 'LineWidth', 1.2);

grid on
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Amplitude', 'FontSize', 12, 'FontWeight', 'bold');
title(sprintf('Low-pass Filtering (f_c = %.1f Hz, T_s = %.3f s)', fc, Ts), ...
      'FontSize', 14, 'FontWeight', 'bold');
legend({'Noisy signal', 'Filtered signal', 'True signal'}, ...
       'Location', 'best', 'FontSize', 11);
set(gca, 'FontSize', 11, 'LineWidth', 1.2);
