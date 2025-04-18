clear; clc; close all;

%% Load Parameters and Plant
run('quanser_aero_parameters.m');
run('quanser_aero_state_space.m');

% Nominal plant
G_unc = ss(A, B, C, D);
G_nom = G_unc.NominalValue;
G_nom.InputName = {'u1', 'u2'};
G_nom.OutputName = {'pitch', 'yaw'};

%% Define Weights
s = tf('s');
wc = 5;         % desired crossover frequency (rad/s)
M = 3;          % max sensitivity
A1 = 100;       % DC gain rejection factor (100 for better RP)
Wp_scalar = (wc)/(s + wc/A1);  % strictly proper, DC gain = A1
Wp = eye(2) * Wp_scalar;
Wp.InputName = {'e1', 'e2'};
Wp.OutputName = {'z1', 'z2'};

Wu = eye(2) * tf(0.01);
Wu.InputName = {'u1', 'u2'};
Wu.OutputName = {'z3', 'z4'};

%% Augment Plant
P_aug = augw(G_nom, Wp, [], Wu);
P_aug.InputName = {'w1', 'w2'};
P_aug.OutputName = {'z1', 'z2', 'z3', 'z4', 'pitch', 'yaw'};

%% H2 Synthesis
nmeas = 2;  % number of measured outputs (pitch, yaw)
ncon  = 2;  % number of control inputs (u1, u2)

[K_h2, CL_h2, gam_h2] = h2syn(P_aug, nmeas, ncon);
disp("H2 optimal gamma: " + gam_h2);

%% Check Performance and Robustness
[Rp, ~, ~] = robustperf(CL_h2);
disp("Robust Performance Margin (RP): " + Rp);

%% Step Response Comparison
T = 5; t = linspace(0, T, 500);
r = [0.2 * ones(1, length(t)); -0.2 * ones(1, length(t))];  % step reference

% Closed-loop system from reference input to output
CL_h2_min = minreal(CL_h2);
[y, ~] = lsim(CL_h2_min(5:6, 1:2), r', t);

figure;
plot(t, y(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t, y(:,2), 'r', 'LineWidth', 1.5);
legend('Pitch', 'Yaw');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Closed-Loop Step Response with H_2 Controller');
grid on;
