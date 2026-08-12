%% MODEL3 - NOMINAL DISCRETE SRV02-ROTFLEX CONTROLLER DESIGN
% Initializes the physical plant, converts it to a 2 ms discrete model,
% maps the desired continuous poles to the z-plane, and computes the
% pole-placement gain used by Q_MDL_DISC.mdl.

clearvars;
close all;
clc;

project_dir = fileparts(mfilename('fullpath'));
modules_dir = fullfile(project_dir, 'Modules');
addpath(modules_dir);
run(fullfile(modules_dir, 'setup_srv02_exp05_rotflex.m'));

Csys = ss(A, B, C, D);
Ts = 0.002;
Dsys = c2d(Csys, Ts, 'zoh');
[AD, BD, CD, DD] = ssdata(Dsys);

zeta = 0.6;
wn = 20;
sigma = zeta*wn;
wd = wn*sqrt(1-zeta^2);

poles_continuous = [-sigma + 1i*wd, ...
                    -sigma - 1i*wd, ...
                    -20, -25];
poles_discrete = exp(poles_continuous*Ts);

K_continuous = place(A, B, poles_continuous);
K = place(AD, BD, poles_discrete);
KI = [1.6, 1.6];

fprintf('Nominal sampling time: %.6f s\n', Ts);
fprintf('Mapped discrete poles:\n');
disp(poles_discrete);
fprintf('Discrete pole-placement gain K:\n');
disp(K);
fprintf('Integral gain KI:\n');
disp(KI);
