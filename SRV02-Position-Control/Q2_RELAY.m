%% Q2_RELAY
% Relay auto-tuning for the identified SRV02 position model.

clearvars;
clc;

tau = 0.0254;       % Identified time constant (s)
K = 1.5286;         % Identified steady-state speed gain ((rad/s)/V)

out = sim('RELAY_TUNING');
t = out.tout(:);
y = out.y(:);
u = out.u(:);

% Use the second half of the simulation to reduce startup-transient bias.
steady_start = max(1, floor(numel(t)/2));
t_steady = t(steady_start:end);
y_steady = y(steady_start:end);
u_steady = u(steady_start:end);

relay_amplitude = (max(u_steady) - min(u_steady)) / 2;
output_amplitude = (max(y_steady) - min(y_steady)) / 2;

[~, peak_indices] = findpeaks(y_steady);
if numel(peak_indices) < 2
    error('At least two steady relay-response peaks are required.');
end

peak_times = t_steady(peak_indices);
period_count = min(4, numel(peak_times) - 1);
Tc = mean(diff(peak_times(end-period_count:end)));

Kc = 4*relay_amplitude / (pi*output_amplitude);
wc = 2*pi / Tc;

Kp = 0.6 * Kc;
Ti = 0.5 * Tc;
Td = 0.125 * Tc;

% Backward-compatible aliases used in the original project notes.
kp = Kp;
ki = Ti;  % Integral time, not parallel-form Ki.
kv = Td;  % Derivative time, not parallel-form Kd.

fprintf('Relay amplitude d = %.6g V\n', relay_amplitude);
fprintf('Output amplitude a = %.6g rad\n', output_amplitude);
fprintf('Critical period Tc = %.6g s\n', Tc);
fprintf('Critical gain Kc = %.6g, critical frequency = %.6g rad/s\n', Kc, wc);
fprintf('Kp = %.6g, Ti = %.6g s, Td = %.6g s\n', Kp, Ti, Td);
