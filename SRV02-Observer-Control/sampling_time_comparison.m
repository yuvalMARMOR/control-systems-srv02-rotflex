%% SAMPLING_TIME_COMPARISON
% Offline illustration of how a controller designed for Ts = 0.002 s
% behaves when it is executed at slower sampling rates. This script does
% not recreate recorded hardware data; the archived figures in assets/
% remain the evidence for the original experiment.

clearvars;
close all;
clc;

project_dir = fileparts(mfilename('fullpath'));
modules_dir = fullfile(project_dir, 'Modules');
addpath(modules_dir);
run(fullfile(modules_dir, 'setup_srv02_exp05_rotflex.m'));

sys_continuous = ss(A, B, C, D);

nominal_Ts = 0.002;
nominal_discrete = c2d(sys_continuous, nominal_Ts, 'zoh');
zeta = 0.6;
wn = 20;
sigma = zeta*wn;
wd = wn*sqrt(1-zeta^2);
desired_continuous_poles = [-sigma + 1i*wd, ...
                            -sigma - 1i*wd, ...
                            -20, -25];
desired_discrete_poles = exp(desired_continuous_poles*nominal_Ts);
K_nominal = place(nominal_discrete.A, nominal_discrete.B, ...
                  desired_discrete_poles);
KI = [1.6, 1.6];

sampling_times = [0.002, 0.05, 0.1];
labels = {'T_s = 0.002 s', 'T_s = 0.05 s', 'T_s = 0.1 s'};
reference = [0.5; 0.0];  % Motor-angle command and zero deflection.
final_time = 5;
voltage_limit = 10;

results = repmat(struct(), 1, numel(sampling_times));
for index = 1:numel(sampling_times)
    Ts_test = sampling_times(index);
    discrete_plant = c2d(sys_continuous, Ts_test, 'zoh');
    results(index) = simulate_sampled_controller( ...
        discrete_plant.A, discrete_plant.B, discrete_plant.C, ...
        K_nominal, KI, Ts_test, final_time, reference, voltage_limit);
    results(index).Ts = Ts_test;
    results(index).state_feedback_eigenvalues = ...
        eig(discrete_plant.A - discrete_plant.B*K_nominal);
end
figure('Name', 'Sampling-time comparison', ...
       'Position', [100, 100, 1250, 820]);

subplot(2, 2, 1);
hold on;
for index = 1:numel(results)
    plot(results(index).t, results(index).y(1, :), ...
         'LineWidth', 1.7, 'DisplayName', labels{index});
end
yline(reference(1), '--k', 'Reference', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Motor-angle response');
legend('Location', 'best');

subplot(2, 2, 2);
hold on;
for index = 1:numel(results)
    plot(results(index).t, results(index).y(2, :), ...
         'LineWidth', 1.7, 'DisplayName', labels{index});
end
yline(reference(2), '--k', 'Reference', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('\alpha (rad)');
title('Flexible-joint deflection');
legend('Location', 'best');

subplot(2, 2, 3);
hold on;
for index = 1:numel(results)
    stairs(results(index).t, results(index).u, ...
           'LineWidth', 1.5, 'DisplayName', labels{index});
end
yline(voltage_limit, ':k');
yline(-voltage_limit, ':k');
grid on;
xlabel('Time (s)');
ylabel('Control voltage (V)');
title('Saturated control signal');
legend('Location', 'best');

subplot(2, 2, 4);
hold on;
unit_circle_angle = linspace(0, 2*pi, 400);
plot(cos(unit_circle_angle), sin(unit_circle_angle), '--k', ...
     'DisplayName', 'Unit circle');
for index = 1:numel(results)
    poles = results(index).state_feedback_eigenvalues;
    plot(real(poles), imag(poles), 'o', 'MarkerSize', 8, ...
         'LineWidth', 1.6, 'DisplayName', labels{index});
end
axis equal;
grid on;
xlabel('Real part');
ylabel('Imaginary part');
title('Fixed-gain state-feedback poles');
legend('Location', 'best');

sgtitle('Offline sampling-time illustration using the nominal 2 ms gain');

fprintf('Nominal controller gain:\n');
disp(K_nominal);
for index = 1:numel(results)
    settling_time = sustained_settling_time( ...
        results(index).y(1, :), reference(1), sampling_times(index), 0.05);
    fprintf('%s: maximum |u| = %.3f V, sustained 5%% settling time = %.3f s\n', ...
            labels{index}, max(abs(results(index).u)), settling_time);
end

function result = simulate_sampled_controller(Ad, Bd, Cd, K, KI, Ts, ...
                                               final_time, reference, ...
                                               voltage_limit)
    sample_count = floor(final_time/Ts) + 1;
    result.t = (0:sample_count-1)*Ts;
    result.x = zeros(4, sample_count);
    result.x_est = zeros(4, sample_count);
    result.y = zeros(2, sample_count);
    result.u = zeros(1, sample_count);
    integral_error = zeros(2, 1);

    for k = 1:sample_count-1
        result.y(:, k) = Cd*result.x(:, k);
        result.x_est(1:2, k) = result.y(:, k);
        if k > 1
            result.x_est(3:4, k) = ...
                (result.y(:, k) - result.y(:, k-1))/Ts;
        end

        error = reference - result.y(:, k);
        integral_error = integral_error + error*Ts;
        raw_control = -K*result.x_est(:, k) + KI*integral_error;
        result.u(k) = min(max(raw_control, -voltage_limit), voltage_limit);
        result.x(:, k+1) = Ad*result.x(:, k) + Bd*result.u(k);
    end

    result.y(:, end) = Cd*result.x(:, end);
    result.x_est(1:2, end) = result.y(:, end);
    result.u(end) = result.u(end-1);
end

function settling_time = sustained_settling_time(signal, reference, Ts, fraction)
    tolerance = fraction*max(abs(reference), eps);
    outside = find(abs(signal - reference) > tolerance);
    if isempty(outside)
        settling_time = 0;
    elseif outside(end) == numel(signal)
        settling_time = NaN;
    else
        settling_time = outside(end)*Ts;
    end
end
