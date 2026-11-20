close all; clear; clc;

% --- IDENTIFIED MODEL (63,2% method, motor 1, Va = 3.84 V) ---
mu  = 2.4080;   % static gain [rad/s per V]
T   = 0.26;     % time constant [s]
tau = 0.06;     % pure time delay [s]

%% --- METHOD 1: OPEN-LOOP ---
% Uses the FOPDT model P(s) = mu*exp(-tau*s)/(1+T*s) already identified:
% no other data is needed, so this file loads no .mat.
% Needs tau > 0 (it appears in a division).
Kp1 = 0.9 * T / (mu * tau);   % proportional gain
Ti1 = 3 * tau;                % integral time [s]
Ki1 = Kp1 / Ti1;              % integral gain, since u = Kp*e + Ki*int(e)

fprintf('--- Ziegler-Nichols, method 1 (open loop) ---\n');
fprintf('Kp = %.4f   Ki = %.4f   (Ti = %.4f s)\n', Kp1, Ki1, Ti1);

%% --- METHOD 2: CLOSED-LOOP ---
% A pure P controller is pushed until the loop oscillates forever: that gain
% is Kcr and the oscillation period is Tcr. Here they are not measured on the
% rig (the firmware runs open loop), but computed from the identified model:
% phase of P(jw) = -pi at wc  ->  Tcr = 2*pi/wc,  Kcr = 1/|P(jwc)|.
assert(tau > 0, 'Method 2 needs a delay: a first-order plant cannot oscillate.');

phase = @(w) -w*tau - atan(w*T) + pi;   % zero where the loop phase is -180 deg
wc  = fzero(phase, [1e-3, 1e4]);        % critical frequency [rad/s]
Kcr = sqrt(1 + (wc*T)^2) / mu;          % = 1/|P(jwc)|
Tcr = 2*pi/wc;                          % critical period [s]

Kp2 = 0.45*Kcr;                         % ZN table, PI row
Ti2 = Tcr/1.2;
Ki2 = Kp2/Ti2;

fprintf('\n--- Ziegler-Nichols, method 2 (closed loop) ---\n');
fprintf('wc = %.2f rad/s   Tcr = %.4f s   Kcr = %.3f   Ti = %.4f s\n', wc, Tcr, Kcr, Ti2);
fprintf('\n');
fprintf('PI from method 1:  Kp = %.4f   Ki = %.4f\n', Kp1, Ki1);
fprintf('PI from method 2:  Kp = %.4f   Ki = %.4f\n', Kp2, Ki2);