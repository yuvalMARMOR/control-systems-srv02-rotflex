%% Q2_ZN
% Ziegler-Nichols reaction-curve analysis for the SRV02 speed model.
%
% If recorded vectors `theta_l` and `t` already exist in the workspace,
% they are used. Otherwise, the identified first-order model is used only
% as an explanatory illustration; that fallback does not reproduce the
% recorded experiment.

tau = 0.0254;       % Identified process time constant (s)
K = 1.5286;         % Identified steady-state speed gain ((rad/s)/V)

has_recorded_data = exist('theta_l', 'var') == 1 && ...
                    exist('t', 'var') == 1 && ...
                    numel(theta_l) == numel(t);

if has_recorded_data
    y = theta_l(:);
    tOut = t(:);
    data_source = 'recorded input vectors';
else
    s = tf('s');
    sys = K / (tau*s + 1);
    [y, tOut] = step(sys);
    y = y(:);
    tOut = tOut(:);
    data_source = 'identified-model illustration';
    warning(['Recorded step-response vectors were not found. ', ...
             'The computed tuning values are illustrative only.']);
end

dy = gradient(y, tOut);
[max_slope, tangent_index] = max(dy);
tangent_time = tOut(tangent_index);
tangent_output = y(tangent_index);

% Parameters used by the documented reaction-curve convention.
a = abs(max_slope*tangent_time - tangent_output);
tau_zn = abs(tangent_time - tangent_output/max_slope);

if a <= sqrt(eps) || tau_zn <= sqrt(eps)
    warning(['The selected response does not contain a well-resolved ', ...
             'reaction-curve delay. Use recorded plant data for tuning.']);
end

Kp = 1.2 / max(a, eps);
Ti = 2.0 * tau_zn;
Td = 0.5 * tau_zn;

% Backward-compatible aliases used in the original project notes.
kp = Kp;
ki = Ti;  % Integral time, not parallel-form Ki.
kv = Td;  % Derivative time, not parallel-form Kd.

tangent = max_slope*(tOut - tangent_time) + tangent_output;
figure('Name', 'Ziegler-Nichols reaction-curve construction');
plot(tOut, y, 'LineWidth', 1.8);
hold on;
plot(tOut, tangent, '--', 'LineWidth', 1.4);
plot(tangent_time, tangent_output, 'o', 'MarkerSize', 7, 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('Response');
legend('Step response', 'Maximum-slope tangent', 'Tangent point', ...
       'Location', 'best');
title('Ziegler-Nichols reaction-curve construction');

fprintf('Data source: %s\n', data_source);
fprintf('a = %.6g, tau_ZN = %.6g s\n', a, tau_zn);
fprintf('Kp = %.6g, Ti = %.6g s, Td = %.6g s\n', Kp, Ti, Td);
