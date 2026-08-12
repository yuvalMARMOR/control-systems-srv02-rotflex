%% OBSERVER_ESTIMATOR_COMPARISON
% Supplemental offline comparison of numerical differentiation, a
% Luenberger observer, and a Kalman filter. The measurement noise in this
% script is synthetic and the resulting plots are explanatory simulations,
% not recorded experimental results.

clearvars;
close all;
clc;

project_dir = fileparts(mfilename('fullpath'));
modules_dir = fullfile(project_dir, 'Modules');
addpath(modules_dir);
run(fullfile(modules_dir, 'setup_srv02_exp05_rotflex.m'));

sys_continuous = ss(A, B, C, D);
Ts = 0.002;
sys_discrete = c2d(sys_continuous, Ts, 'zoh');
Ad = sys_discrete.A;
Bd = sys_discrete.B;
Cd = sys_discrete.C;

zeta = 0.6;
wn = 20;
sigma = zeta*wn;
wd = wn*sqrt(1-zeta^2);
controller_poles_s = [-sigma + 1i*wd, -sigma - 1i*wd, -20, -25];
controller_poles_z = exp(controller_poles_s*Ts);
K = place(Ad, Bd, controller_poles_z);

observer_poles_s = [-55, -60, -65, -70];
observer_poles_z = exp(observer_poles_s*Ts);
L = place(Ad', Cd', observer_poles_z)';

KI = [1.6, 1.6];
reference = [0.5; 0.0];
final_time = 5;
voltage_limit = 10;
sample_count = floor(final_time/Ts) + 1;

rng(7, 'twister');
measurement_noise_std = 0.002;
measurement_noise = measurement_noise_std*randn(2, sample_count);

numerical = simulate_estimator('numerical', Ad, Bd, Cd, K, L, KI, ...
    Ts, final_time, reference, voltage_limit, measurement_noise);
luenberger = simulate_estimator('luenberger', Ad, Bd, Cd, K, L, KI, ...
    Ts, final_time, reference, voltage_limit, measurement_noise);
kalman = simulate_estimator('kalman', Ad, Bd, Cd, K, L, KI, ...
    Ts, final_time, reference, voltage_limit, measurement_noise);

methods = {'Numerical derivative', 'Luenberger observer', 'Kalman filter'};
data = {numerical, luenberger, kalman};
colors = lines(3);

figure('Name', 'State-estimator comparison', ...
       'Position', [100, 100, 1300, 850]);

subplot(2, 2, 1);
hold on;
for index = 1:3
    plot(data{index}.t, data{index}.y(1, :), 'LineWidth', 1.5, ...
         'Color', colors(index, :), 'DisplayName', methods{index});
end
yline(reference(1), '--k', 'Reference');
grid on;
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Motor-angle response');
legend('Location', 'best');

subplot(2, 2, 2);
hold on;
plot(numerical.t, numerical.x(3, :), 'k', 'LineWidth', 1.5, ...
     'DisplayName', 'True angular velocity');
for index = 1:3
    plot(data{index}.t, data{index}.x_est(3, :), 'LineWidth', 1.2, ...
         'Color', colors(index, :), 'DisplayName', methods{index});
end
grid on;
xlabel('Time (s)');
ylabel('d\theta/dt (rad/s)');
title('Estimated motor angular velocity');
legend('Location', 'best');

subplot(2, 2, 3);
hold on;
for index = 1:3
    error = data{index}.x(3, :) - data{index}.x_est(3, :);
    plot(data{index}.t, error, 'LineWidth', 1.2, ...
         'Color', colors(index, :), 'DisplayName', methods{index});
end
grid on;
xlabel('Time (s)');
ylabel('Estimation error (rad/s)');
title('Motor-velocity estimation error');
legend('Location', 'best');

subplot(2, 2, 4);
hold on;
for index = 1:3
    stairs(data{index}.t, data{index}.u, 'LineWidth', 1.2, ...
           'Color', colors(index, :), 'DisplayName', methods{index});
end
yline(voltage_limit, ':k');
yline(-voltage_limit, ':k');
grid on;
xlabel('Time (s)');
ylabel('Control voltage (V)');
title('Control effort');
legend('Location', 'best');

sgtitle(sprintf(['Supplemental estimator comparison with synthetic ', ...
                 'measurement noise (standard deviation %.4f rad)'], ...
                 measurement_noise_std));

fprintf('Synthetic-noise estimator comparison:\n');
for index = 1:3
    velocity_error = data{index}.x(3, :) - data{index}.x_est(3, :);
    rmse = sqrt(mean(velocity_error.^2));
    settling_time = sustained_settling_time( ...
        data{index}.y(1, :), reference(1), Ts, 0.05);
    fprintf('%s: velocity RMSE = %.6f rad/s, settling time = %.3f s\n', ...
            methods{index}, rmse, settling_time);
end
function result = simulate_estimator(method, Ad, Bd, Cd, K, L, KI, ...
                                     Ts, final_time, reference, ...
                                     voltage_limit, measurement_noise)
    sample_count = floor(final_time/Ts) + 1;
    result.t = (0:sample_count-1)*Ts;
    result.x = zeros(4, sample_count);
    result.x_est = zeros(4, sample_count);
    result.y = zeros(2, sample_count);
    result.u = zeros(1, sample_count);
    integral_error = zeros(2, 1);

    process_covariance = diag([1e-7, 1e-7, 1e-5, 1e-5]);
    measurement_covariance = diag([4e-6, 4e-6]);
    covariance = eye(4);

    for k = 1:sample_count-1
        measured_output = Cd*result.x(:, k) + measurement_noise(:, k);
        result.y(:, k) = measured_output;

        switch method
            case 'numerical'
                result.x_est(1:2, k) = measured_output;
                if k > 1
                    result.x_est(3:4, k) = ...
                        (measured_output - result.y(:, k-1))/Ts;
                end
                estimate_for_control = result.x_est(:, k);

            case 'luenberger'
                estimate_for_control = result.x_est(:, k);

            case 'kalman'
                innovation_covariance = ...
                    Cd*covariance*Cd' + measurement_covariance;
                kalman_gain = covariance*Cd'/innovation_covariance;
                corrected_estimate = result.x_est(:, k) + ...
                    kalman_gain*(measured_output - Cd*result.x_est(:, k));
                covariance = (eye(4) - kalman_gain*Cd)*covariance;
                result.x_est(:, k) = corrected_estimate;
                estimate_for_control = corrected_estimate;

            otherwise
                error('Unknown estimator method: %s', method);
        end

        tracking_error = reference - measured_output;
        integral_error = integral_error + tracking_error*Ts;
        raw_control = -K*estimate_for_control + KI*integral_error;
        result.u(k) = min(max(raw_control, -voltage_limit), voltage_limit);
        result.x(:, k+1) = Ad*result.x(:, k) + Bd*result.u(k);

        switch method
            case 'numerical'
                result.x_est(:, k+1) = result.x_est(:, k);
            case 'luenberger'
                innovation = measured_output - Cd*result.x_est(:, k);
                result.x_est(:, k+1) = Ad*result.x_est(:, k) + ...
                    Bd*result.u(k) + L*innovation;
            case 'kalman'
                result.x_est(:, k+1) = ...
                    Ad*estimate_for_control + Bd*result.u(k);
                covariance = Ad*covariance*Ad' + process_covariance;
        end
    end

    result.y(:, end) = Cd*result.x(:, end) + measurement_noise(:, end);
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
