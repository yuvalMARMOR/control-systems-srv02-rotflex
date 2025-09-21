%% ????? ???? ???????? ?? Observer ????? ?????? ????? ??? ?? ??????
% Digital Control System with Advanced Observer for State Estimation
% ???? Luenberger Observer ?-Kalman Filter

clear all; close all; clc;

%% ????? ?????? ??????
% System Parameters Definition

% ????? SRV02
EXT_GEAR_CONFIG = 'HIGH';
ENCODER_TYPE = 'E';
TACH_OPTION = 'NO';
AMP_TYPE = 'UPM_1503';
LOAD_TYPE = 'ROTFLEX';

% ????? ROTFLEX
SPRING = 2;
HUB_POSITION = 'B';
ARM_POSITION = 2;
ARM_LOAD = 0;
ROTFLEX_OPTION = 'ROTFLEX';

% ???? ?????? ??????
[Rm, kt, km, Kg, eta_g, Beq, Jm, Jeq, eta_m, K_POT, K_TACH, K_ENC, VMAX_AMP, IMAX_AMP] = ...
    setup_srv02_configuration(EXT_GEAR_CONFIG, ENCODER_TYPE, TACH_OPTION, AMP_TYPE, LOAD_TYPE);

[Jarm, Barm, Ks, RtflxOp, K_POT_ROT] = config_rotflex(SPRING, HUB_POSITION, ARM_POSITION, ARM_LOAD, ROTFLEX_OPTION);

% ????? ????? ??????
K_Stiff = Ks;
Jeq_total = Jeq + Jarm;

%% ????? ??????? ????
% State-Space Matrices Construction

% ?????? ?????? A
A = zeros(4,4);
A(1,3) = 1;
A(2,4) = 1;
A(3,2) = K_Stiff/Jeq;
A(3,3) = -(eta_g*Kg^2*eta_m*kt*km + Beq*Rm)/(Rm*Jeq);
A(4,2) = -K_Stiff*(Jeq + Jarm)/(Jeq*Jarm);
A(4,3) = (eta_g*Kg^2*eta_m*kt*km + Beq*Rm)/(Rm*Jeq);

% ?????? ?????? B
B = zeros(4,1);
B(3) = eta_g*Kg*eta_m*kt/(Rm*Jeq);
B(4) = -eta_g*Kg*eta_m*kt/(Rm*Jeq);

% ?????? ???? C - ?? ??????? ??????
C = zeros(2,4);
C(1,1) = 1;  % ?
C(2,2) = 1;  % ?

% ?????? ????? ?????? D
D = zeros(2,1);

% ????? ???? ????
sys_cont = ss(A, B, C, D);

%% ????? ?????? ?????
% Sampling Intervals Definition

Ts_values = [0.002, 0.05, 0.1];
Ts_names = {'Ts = 0.002s', 'Ts = 0.05s', 'Ts = 0.1s'};

%% ????? Observer
% Observer Design

% ????? ?????? ??????
p_cont = [-10, -15, -20, -25];

% ????? ?????? ?Observer (?????? ???? ???????)
p_obs = [-30, -35, -40, -45];

%% ???????? ???????
% Simulation and Comparison

t_final = 5;
ref_theta = 0.5;
ref_alpha = 0.3;

results_obs = struct();

for i = 1:length(Ts_values)
    Ts = Ts_values(i);
    disp(['???? ???????? ?? Observer - ', Ts_names{i}, '...']);
    
    % ???? ????? ????
    sys_disc = c2d(sys_cont, Ts, 'zoh');
    Ad = sys_disc.A;
    Bd = sys_disc.B;
    Cd = sys_disc.C;
    Dd = sys_disc.D;
    
    % ????? ??????
    p_disc = exp(p_cont * Ts);
    p_obs_disc = exp(p_obs * Ts);
    
    % ????? ???
    K_disc = place(Ad, Bd, p_disc);
    
    % ????? Observer
    L_disc = place(Ad', Cd', p_obs_disc)';
    
    % ????? ??? ????????
    KI_disc = [1.6, 1.6] * Ts;
    
    % ???????? ?? Observer
    [t_obs, y_obs, x_obs, x_est_obs] = simulate_with_observer(Ad, Bd, Cd, K_disc, L_disc, KI_disc, Ts, t_final, ref_theta, ref_alpha);
    
    % ????? ??????
    results_obs(i).Ts = Ts;
    results_obs(i).t = t_obs;
    results_obs(i).y = y_obs;
    results_obs(i).x = x_obs;
    results_obs(i).x_est = x_est_obs;
    results_obs(i).K = K_disc;
    results_obs(i).L = L_disc;
    results_obs(i).KI = KI_disc;
end

%% ?????? ??? ????? ????? ????? ???
% Comparison of State Estimation Methods

Ts_test = 0.05;
sys_disc_test = c2d(sys_cont, Ts_test, 'zoh');

% ????? ????? ?Observers
p_disc_test = exp(p_cont * Ts_test);
p_obs_disc_test = exp(p_obs * Ts_test);
K_test = place(sys_disc_test.A, sys_disc_test.B, p_disc_test);
L_test = place(sys_disc_test.A', sys_disc_test.C', p_obs_disc_test)';
KI_test = [1.6, 1.6] * Ts_test;

% ???????? ?? ??????????? ??????
[t_num, y_num, x_num, x_est_num] = simulate_numerical_differentiation(sys_disc_test.A, sys_disc_test.B, sys_disc_test.C, K_test, KI_test, Ts_test, t_final, ref_theta, ref_alpha);

% ???????? ?? Observer
[t_obs, y_obs, x_obs, x_est_obs] = simulate_with_observer(sys_disc_test.A, sys_disc_test.B, sys_disc_test.C, K_test, L_test, KI_test, Ts_test, t_final, ref_theta, ref_alpha);

% ???????? ?? Kalman Filter
[t_kf, y_kf, x_kf, x_est_kf] = simulate_with_kalman_filter(sys_disc_test.A, sys_disc_test.B, sys_disc_test.C, K_test, KI_test, Ts_test, t_final, ref_theta, ref_alpha);

%% ???? ?????? ??????
% Comparison Results Display

figure('Position', [100, 100, 1400, 1000]);

% ?????? ??????
subplot(3,3,1);
hold on;
plot(t_num, y_num(1,:), 'b-', 'LineWidth', 2, 'DisplayName', '??????????? ??????');
plot(t_obs, y_obs(1,:), 'r-', 'LineWidth', 2, 'DisplayName', 'Observer');
plot(t_kf, y_kf(1,:), 'g-', 'LineWidth', 2, 'DisplayName', 'Kalman Filter');
plot(t_num, ref_theta * ones(size(t_num)), '--k', 'LineWidth', 2, 'DisplayName', '?????');
xlabel('??? (?????)');
ylabel('? (?????)');
title('????? ????? - ?????? ?????');
legend('Location', 'best');
grid on;

subplot(3,3,2);
hold on;
plot(t_num, y_num(2,:), 'b-', 'LineWidth', 2, 'DisplayName', '??????????? ??????');
plot(t_obs, y_obs(2,:), 'r-', 'LineWidth', 2, 'DisplayName', 'Observer');
plot(t_kf, y_kf(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'Kalman Filter');
plot(t_num, ref_alpha * ones(size(t_num)), '--k', 'LineWidth', 2, 'DisplayName', '?????');
xlabel('??? (?????)');
ylabel('? (?????)');
title('????? ????? - ?????? ?????');
legend('Location', 'best');
grid on;

% ?????? ???????? (??????? vs ???????)
subplot(3,3,3);
hold on;
plot(t_num, x_num(3,:), 'k-', 'LineWidth', 2, 'DisplayName', '?????');
plot(t_num, x_est_num(3,:), 'b--', 'LineWidth', 2, 'DisplayName', '??????????? ??????');
plot(t_obs, x_est_obs(3,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Observer');
plot(t_kf, x_est_kf(3,:), 'g--', 'LineWidth', 2, 'DisplayName', 'Kalman Filter');
xlabel('??? (?????)');
ylabel('?? (?????/?????)');
title('?????? ????? - ????? vs ?????');
legend('Location', 'best');
grid on;

% ?????? ?????
subplot(3,3,4);
hold on;
error_num = x_num(3,:) - x_est_num(3,:);
error_obs = x_obs(3,:) - x_est_obs(3,:);
error_kf = x_kf(3,:) - x_est_kf(3,:);
plot(t_num, error_num, 'b-', 'LineWidth', 2, 'DisplayName', '??????????? ??????');
plot(t_obs, error_obs, 'r-', 'LineWidth', 2, 'DisplayName', 'Observer');
plot(t_kf, error_kf, 'g-', 'LineWidth', 2, 'DisplayName', 'Kalman Filter');
xlabel('??? (?????)');
ylabel('????? ????? ??');
title('?????? ????? - ?????? ?????');
legend('Location', 'best');
grid on;

% ????? ?????? Observer
subplot(3,3,5);
hold on;
for i = 1:length(Ts_values)
    Ts = Ts_values(i);
    sys_disc = c2d(sys_cont, Ts, 'zoh');
    p_obs_disc = exp(p_obs * Ts);
    L_disc = place(sys_disc.A', sys_disc.C', p_obs_disc)';
    
    % ???? Observer
    A_obs = sys_disc.A - L_disc * sys_disc.C;
    eig_obs = eig(A_obs);
    
    plot(real(eig_obs), imag(eig_obs), 'o', 'MarkerSize', 10, 'LineWidth', 2, ...
         'DisplayName', Ts_names{i});
end

theta = 0:0.01:2*pi;
plot(cos(theta), sin(theta), '--k', 'LineWidth', 1, 'DisplayName', '???? ??????');
xlabel('Real Part');
ylabel('Imaginary Part');
title('???? Observer ????? Z');
legend('Location', 'best');
grid on;
axis equal;

% ????? ?????? ????
subplot(3,3,6);
hold on;
noise_levels = [0, 0.001, 0.005, 0.01];
Ts_test = 0.05;

for noise_level = noise_levels
    [t_noise, y_noise, x_noise, x_est_noise] = simulate_observer_with_noise(sys_cont, Ts_test, t_final, ref_theta, ref_alpha, noise_level);
    plot(t_noise, y_noise(1,:), 'LineWidth', 2, 'DisplayName', ['??? = ', num2str(noise_level)]);
end

xlabel('??? (?????)');
ylabel('? (?????)');
title('?????? Observer ????');
legend('Location', 'best');
grid on;

% ?????? ???????
subplot(3,3,7);
methods = {'??????????? ??????', 'Observer', 'Kalman Filter'};
rmse_theta = [sqrt(mean((x_num(3,:) - x_est_num(3,:)).^2)), ...
              sqrt(mean((x_obs(3,:) - x_est_obs(3,:)).^2)), ...
              sqrt(mean((x_kf(3,:) - x_est_kf(3,:)).^2))];
bar(rmse_theta);
set(gca, 'XTickLabel', methods);
ylabel('RMSE ?? ??');
title('?????? ???? ?????');
grid on;

% ??? ???????
subplot(3,3,8);
settling_times = [find_settling_time(y_num(1,:), ref_theta, Ts_test), ...
                  find_settling_time(y_obs(1,:), ref_theta, Ts_test), ...
                  find_settling_time(y_kf(1,:), ref_theta, Ts_test)];
bar(settling_times);
set(gca, 'XTickLabel', methods);
ylabel('??? ??????? (?????)');
title('?????? ??? ???????');
grid on;

% ????? ????
subplot(3,3,9);
hold on;
u_num = calculate_control_signal(x_est_num, K_test, KI_test, ref_theta, ref_alpha);
u_obs = calculate_control_signal(x_est_obs, K_test, KI_test, ref_theta, ref_alpha);
u_kf = calculate_control_signal(x_est_kf, K_test, KI_test, ref_theta, ref_alpha);

plot(t_num, u_num, 'b-', 'LineWidth', 2, 'DisplayName', '??????????? ??????');
plot(t_obs, u_obs, 'r-', 'LineWidth', 2, 'DisplayName', 'Observer');
plot(t_kf, u_kf, 'g-', 'LineWidth', 2, 'DisplayName', 'Kalman Filter');
xlabel('??? (?????)');
ylabel('??? ???? (V)');
title('????? ???? - ??????');
legend('Location', 'best');
grid on;

sgtitle('?????? ????? ????? ????? ??? ?? ??????', 'FontSize', 16);

%% ???????? ???
% Helper Functions

function [t, y, x, x_est] = simulate_with_observer(Ad, Bd, Cd, K, L, KI, Ts, t_final, ref_theta, ref_alpha)
    % ???????? ?? Luenberger Observer
    
    N = ceil(t_final/Ts);
    t = (0:N-1) * Ts;
    
    x = zeros(4, N);
    y = zeros(2, N);
    u = zeros(1, N);
    x_est = zeros(4, N);
    
    int_error_theta = 0;
    int_error_alpha = 0;
    
    % ???? ?????
    x(:,1) = [0; 0; 0; 0];
    x_est(:,1) = [0; 0; 0; 0];
    
    for k = 1:N-1
        % ??????
        y(:,k) = Cd * x(:,k);
        
        % ????? ??????
        error_theta = ref_theta - y(1,k);
        error_alpha = ref_alpha - y(2,k);
        
        % ????? ????????
        int_error_theta = int_error_theta + error_theta * Ts;
        int_error_alpha = int_error_alpha + error_alpha * Ts;
        
        % ????? Observer
        y_pred = Cd * x_est(:,k);
        x_est(:,k+1) = Ad * x_est(:,k) + Bd * u(k) + L * (y(:,k) - y_pred);
        
        % ??? ????
        u(k) = K * x_est(:,k) + KI(1) * int_error_theta + KI(2) * int_error_alpha;
        
        % ????? ??? ??????
        x(:,k+1) = Ad * x(:,k) + Bd * u(k);
    end
    
    y(:,N) = Cd * x(:,N);
end

function [t, y, x, x_est] = simulate_numerical_differentiation(Ad, Bd, Cd, K, KI, Ts, t_final, ref_theta, ref_alpha)
    % ???????? ?? ??????????? ??????
    
    N = ceil(t_final/Ts);
    t = (0:N-1) * Ts;
    
    x = zeros(4, N);
    y = zeros(2, N);
    u = zeros(1, N);
    x_est = zeros(4, N);
    
    int_error_theta = 0;
    int_error_alpha = 0;
    
    x(:,1) = [0; 0; 0; 0];
    x_est(:,1) = [0; 0; 0; 0];
    
    for k = 1:N-1
        y(:,k) = Cd * x(:,k);
        
        error_theta = ref_theta - y(1,k);
        error_alpha = ref_alpha - y(2,k);
        
        int_error_theta = int_error_theta + error_theta * Ts;
        int_error_alpha = int_error_alpha + error_alpha * Ts;
        
        % ??????????? ??????
        if k > 1
            x_est(3,k) = (y(1,k) - y(1,k-1)) / Ts;
            x_est(4,k) = (y(2,k) - y(2,k-1)) / Ts;
        end
        
        x_est(1,k) = y(1,k);
        x_est(2,k) = y(2,k);
        
        u(k) = K * x_est(:,k) + KI(1) * int_error_theta + KI(2) * int_error_alpha;
        x(:,k+1) = Ad * x(:,k) + Bd * u(k);
    end
    
    y(:,N) = Cd * x(:,N);
end

function [t, y, x, x_est] = simulate_with_kalman_filter(Ad, Bd, Cd, K, KI, Ts, t_final, ref_theta, ref_alpha)
    % ???????? ?? Kalman Filter
    
    N = ceil(t_final/Ts);
    t = (0:N-1) * Ts;
    
    x = zeros(4, N);
    y = zeros(2, N);
    u = zeros(1, N);
    x_est = zeros(4, N);
    
    % ??????? ???
    Q = diag([0.001, 0.001, 0.01, 0.01]);  % ??? ?????
    R = diag([0.001, 0.001]);              % ??? ?????
    
    % ????? Kalman Filter
    P = eye(4);  % ?????? ????? ???????
    x_est(:,1) = [0; 0; 0; 0];
    
    int_error_theta = 0;
    int_error_alpha = 0;
    
    for k = 1:N-1
        y(:,k) = Cd * x(:,k);
        
        error_theta = ref_theta - y(1,k);
        error_alpha = ref_alpha - y(2,k);
        
        int_error_theta = int_error_theta + error_theta * Ts;
        int_error_alpha = int_error_alpha + error_alpha * Ts;
        
        % ????? Kalman
        K_kalman = P * Cd' * inv(Cd * P * Cd' + R);
        
        % ?????
        x_est(:,k) = x_est(:,k) + K_kalman * (y(:,k) - Cd * x_est(:,k));
        P = (eye(4) - K_kalman * Cd) * P;
        
        % ?????
        u(k) = K * x_est(:,k) + KI(1) * int_error_theta + KI(2) * int_error_alpha;
        x_est(:,k+1) = Ad * x_est(:,k) + Bd * u(k);
        P = Ad * P * Ad' + Q;
        
        x(:,k+1) = Ad * x(:,k) + Bd * u(k);
    end
    
    y(:,N) = Cd * x(:,N);
end

function [t, y, x, x_est] = simulate_observer_with_noise(sys_cont, Ts, t_final, ref_theta, ref_alpha, noise_level)
    % ???????? ?? Observer ????
    
    sys_disc = c2d(sys_cont, Ts, 'zoh');
    p_cont = [-10, -15, -20, -25];
    p_obs = [-30, -35, -40, -45];
    
    p_disc = exp(p_cont * Ts);
    p_obs_disc = exp(p_obs * Ts);
    
    K_disc = place(sys_disc.A, sys_disc.B, p_disc);
    L_disc = place(sys_disc.A', sys_disc.C', p_obs_disc)';
    KI_disc = [1.6, 1.6] * Ts;
    
    [t, y, x, x_est] = simulate_with_observer(sys_disc.A, sys_disc.B, sys_disc.C, K_disc, L_disc, KI_disc, Ts, t_final, ref_theta, ref_alpha);
    
    % ????? ??? ???????
    for k = 1:length(t)
        y(:,k) = y(:,k) + noise_level * randn(2,1);
    end
end

function settling_time = find_settling_time(y, ref, Ts)
    % ????? ??? ???????
    threshold = 0.05 * abs(ref);
    settled_indices = find(abs(y - ref) < threshold);
    if ~isempty(settled_indices)
        settling_time = settled_indices(1) * Ts;
    else
        settling_time = NaN;
    end
end

function u = calculate_control_signal(x_est, K, KI, ref_theta, ref_alpha)
    % ????? ??? ?????
    u = zeros(1, size(x_est, 2));
    int_error_theta = 0;
    int_error_alpha = 0;
    
    for k = 1:size(x_est, 2)
        if k > 1
            error_theta = ref_theta - x_est(1,k);
            error_alpha = ref_alpha - x_est(2,k);
            int_error_theta = int_error_theta + error_theta * 0.05;  % Ts = 0.05
            int_error_alpha = int_error_alpha + error_alpha * 0.05;
        end
        u(k) = K * x_est(:,k) + KI(1) * int_error_theta + KI(2) * int_error_alpha;
    end
end

%% ????? ??????
% Results Summary

disp('=== ????? ?????? ????? ????? ????? ??? ===');
disp('');

% ????? ???????
rmse_theta_dot = [sqrt(mean((x_num(3,:) - x_est_num(3,:)).^2)), ...
                  sqrt(mean((x_obs(3,:) - x_est_obs(3,:)).^2)), ...
                  sqrt(mean((x_kf(3,:) - x_est_kf(3,:)).^2))];

settling_times = [find_settling_time(y_num(1,:), ref_theta, Ts_test), ...
                  find_settling_time(y_obs(1,:), ref_theta, Ts_test), ...
                  find_settling_time(y_kf(1,:), ref_theta, Ts_test)];

methods = {'??????????? ??????', 'Observer', 'Kalman Filter'};


for i = 1:3
    disp([methods{i}, ':']);
    disp(['  RMSE ?? ??: ', num2str(rmse_theta_dot(i), '%.6f')]);
    disp(['  ??? ???????: ', num2str(settling_times(i), '%.3f'), ' ?????']);
    disp('');
end

disp('??????:');
disp('1. Observer ????? ?????? ?? ??? ????');
disp('2. Kalman Filter ????? ?????? ?? ??? ????');
disp('3. ??????????? ?????? ????? ?? ????? ????');
disp('4. ???? ??????????? ?? ?????? ???? ?-Kalman Filter');