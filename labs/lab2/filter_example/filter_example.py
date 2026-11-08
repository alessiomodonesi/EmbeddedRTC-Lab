import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# === Filter parameters ===
fc = 1.0       # Cutoff frequency [Hz]
Ts = 0.01      # Sampling period [s]

# === Define an example noisy signal ===
t = np.arange(0, 2 + Ts, Ts)         # Time vector [s]
y_true = 1 - np.exp(-t / 0.2)        # Original signal
noise = 0.02 * np.random.randn(len(y_true))  # Additive white Gaussian noise
y = y_true + noise                   # Noisy signal

# === Define the continuous-time low-pass filter ===
# The analog (continuous-time) transfer function of a first-order LPF is:
#
#              (2*pi*fc)
#   H(s) = -----------------
#          (s + 2*pi*fc)
#
# This is a standard RC-like first-order low-pass filter.
num = [2 * np.pi * fc]               # Numerator coefficients (s^0)
den = [1, 2 * np.pi * fc]            # Denominator coefficients (s^1 + constant)
sysF = signal.lti(num, den)          # Continuous-time transfer function (Laplace domain)

# === Discretize the filter (continuous -> discrete) ===
# The signal y is discrete (sampled every Ts seconds),
# while sysF is continuous-time (defined in variable s).
#
# To apply it digitally, we convert H(s) → H(z) using the Tustin (bilinear) transform.
#
# The bilinear transform replaces:
#     s = (2/Ts) * (1 - z^-1) / (1 + z^-1)
#
# The Tustin transform:
#   * Preserves the frequency response well at low frequencies
#   * Maps the stable left-half s-plane to the inside of the unit circle (stable in z)
#   * Avoids aliasing of poles and zeros
#   * Produces a stable, causal discrete-time filter suitable for implementation
#
# The resulting discrete filter H(z) ≈ H(s) in behavior for frequencies
# below Nyquist (Fs/2).
sysF_d = signal.cont2discrete((num, den), Ts, method='bilinear')

# Extract discrete-time filter coefficients
numF = sysF_d[0].flatten()
denF = sysF_d[1].flatten()

# === Apply the digital filter to the noisy signal ===
yf = signal.lfilter(numF, denF, y)

# === Plot the results ===
plt.figure(figsize=(8, 4), facecolor='white')
plt.plot(t, y, color='g', linewidth=1.4, label='Noisy signal')
plt.plot(t, yf, color='b', linewidth=1.8, label='Filtered signal')
plt.plot(t, y_true, 'r--', linewidth=1.2, label='True signal')

plt.grid(True, which='both', linestyle='--', linewidth=0.6)
plt.xlabel('Time [s]', fontsize=12, fontweight='bold')
plt.ylabel('Amplitude', fontsize=12, fontweight='bold')
plt.title(f'Low-pass Filtering (f_c = {fc:.1f} Hz, T_s = {Ts:.3f} s)',
          fontsize=14, fontweight='bold')
plt.legend(fontsize=10, loc='best')
plt.tight_layout()
plt.show()
