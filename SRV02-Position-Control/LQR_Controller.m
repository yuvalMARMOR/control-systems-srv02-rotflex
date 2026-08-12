%% LQR_CONTROLLER
% Computes the LQR gain used for the SRV02 position-control study.
% The identified first-order speed-model parameters are defined explicitly
% so that this script does not depend on an undocumented base workspace.

clearvars;
clc;

tau = 0.0254;       % Identified time constant (s)
K = 1.5286;         % Identified steady-state speed gain ((rad/s)/V)

% State order: load angle, load angular velocity, integral state.
A = [0, 1,      0;
     0, -1/tau, 0;
     1, 0,      0];
B = [0; K/tau; 0];

Q = diag([30, 0, 3]);
R = 1;

[K1, S, closed_loop_poles] = lqr(A, B, Q, R);

fprintf('LQR gain [position, velocity, integral]:\n');
disp(K1);
fprintf('Closed-loop poles:\n');
disp(closed_loop_poles);
