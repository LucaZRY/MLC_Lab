%% Part E: μ-Synthesis Controller Design (MLC Lab Aero 2025)
clear; clc; close all;

% Load Plant Parameters and State-Space Model
run('quanser_aero_parameters.m');
run('quanser_aero_state_space.m');

% Create uncertain plant
G_unc = ss(A, B, C, D);     % Nominal plant
G_unc = uss(G_unc);         % Convert to uncertain model
G_unc.InputName = {'u1','u2'};
G_unc.OutputName = {'y1','y2'};

% Sample and Fit Input Multiplicative Uncertainty
G_samples = usample(G_unc, 300);
[~, info] = ucover(G_samples, G_unc.NominalValue, 4, [], 'InputMult');
W_I = info.W1;

% Define dynamic uncertainty block
Delta = ultidyn('Delta', [2 2], 'Bound', 1);

% Define uncertain perturbed plant: G_unc * (I + W_I * Delta)
G_pert = G_unc * (eye(2) + W_I * Delta);
G_pert.InputName = {'u1','u2'};
G_pert.OutputName = {'y1','y2'};

% Define weighting functions (same as H∞ design)
s = tf('s');
Wp = blkdiag(1000/(s + 0.33), 1000/(s + 0.33));
Wu = blkdiag(tf(0.1), tf(0.1));

Wp.InputName = {'y1','y2'};
Wp.OutputName = {'ep1','ep2'};

Wu.InputName = {'u1','u2'};
Wu.OutputName = {'eu1','eu2'};

% Define summing blocks for reference tracking
Sum1 = sumblk('e1 = r1 - y1');
Sum2 = sumblk('e2 = r2 - y2');

% Build generalized plant for musyn
% Inputs: [r1; r2; u1; u2]
% Outputs: [ep1; ep2; eu1; eu2; y1; y2]
P_mu = connect(G_pert, Wp, Wu, Sum1, Sum2, ...
    {'r1','r2','u1','u2'}, {'ep1','ep2','eu1','eu2','y1','y2'});

% μ-Synthesis
nmeas = 2; % y1, y2
ncon = 2;  % u1, u2

[K_mu, CL_mu, info_mu] = musyn(P_mu, nmeas, ncon);

% Optional: Reduce controller order
K_mu = balred(K_mu, order(K_mu));

% Save controller
save('K_mu.mat', 'K_mu');

% Bode plot of controller
figure;
bode(K_mu), grid on;
title('Bode Plot of \mu-Synthesis Controller');

% Get closed-loop from reference (r) to output (y)
CL_r2y = lft(P_mu, K_mu);  % Already r → y if I/O labeled correctly

% Simulate step response (Δ = 0)
t = 0:0.01:10;
r = ones(length(t), 2);  % Step input

y = lsim(CL_r2y, r, t);

% Plot step response
figure;
plot(t, y(:,1), 'b', t, y(:,2), 'r');
grid on;
xlabel('Time (s)');
ylabel('Output');
title('Closed-Loop Step Response with \mu-Controller (Δ = 0)');
legend('Pitch', 'Yaw');

% Robust Stability & Performance (only for performance outputs)
CL_perf = CL_mu(1:4, :);  % Extract outputs: ep1, ep2, eu1, eu2

[stabmarg, ~, ~] = robuststab(CL_perf);
disp("Robust Stability Margin: " + num2str(stabmarg.LowerBound));

[perfmarg, ~, ~] = robustperf(CL_perf);
disp("Robust Performance Margin: " + num2str(perfmarg.LowerBound));

