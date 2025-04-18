clear; clc; close all;

%% Load uncertain plant model
quanser_aero_parameters;
quanser_aero_state_space;

G_unc = ss(A, B, C, D); 
% uncertain plant
G_nom = G_unc.NominalValue;   % nominal for synthesis

%% Define weights (same as H∞ design)
s = tf('s');
wc = 5;        % crossover freq
M = 3;         % sensitivity peak
A = 1/1000;    % low-frequency disturbance rejection

Wp_single = (s/M + wc) / (s + wc*A);
Wp = blkdiag(Wp_single, Wp_single);
Wu = 0.01 * eye(2);  % constant control weight

%% Build uncertain generalized plant for μ-synthesis
systemnames = 'G_unc Wp Wu';
inputvar = '[w(2); u(2)]';
outputvar = '[Wp; Wu; w-G_unc]';
input_to_G_unc = '[u]';
input_to_Wp = '[w-G_unc]';
input_to_Wu = '[u]';
P_unc = sysic;

%% μ-synthesis
omega = logspace(-2, 3, 100);
delta = ultidyn('delta', [1 1]);
[K_mu, CL_mu, bnd, dkinfo] = dksyn(P_unc, 2, 2);
disp("μ-synthesis completed with peak μ value: " + bnd(1));

%% Reduce controller order to match H∞ controller
K_mu_red = reduce(K_mu, size(K_hinf, 3));

%% Closed-loop from r to y (nominal)
T_mu = feedback(G_nom * K_mu_red, eye(2));

%% Simulate tracking
t = 0:0.01:10;
r = [pi/6 * ones(length(t),1), pi/4 * ones(length(t),1)];
[y_mu, ~] = lsim(T_mu, r, t);

% Compute control input: u = K * (r - y)
e = r - y_mu;
u_mu = lsim(K_mu_red, e, t);

%% Plot outputs
figure;
subplot(2,1,1);
plot(t, y_mu(:,1), 'LineWidth', 1.5); hold on;
plot(t, y_mu(:,2), 'LineWidth', 1.5);
title('μ-Synthesis Output Response (Angles)');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('Pitch','Yaw'); grid on;

%% Plot control inputs
subplot(2,1,2);
plot(t, u_mu(:,1), 'LineWidth', 1.5); hold on;
plot(t, u_mu(:,2), 'LineWidth', 1.5);
title('μ-Synthesis Control Inputs');
xlabel('Time (s)'); ylabel('Voltage (V)');
legend('u1','u2'); grid on;

%% === Robust Stability Analysis ===
CL_unc_mu = feedback(G_unc * K_mu_red, eye(2));
[stab_margin_mu, stab_report_mu] = robstab(CL_unc_mu);
disp('--- Robust Stability (μ) ---');
RS_mu = 1 / stab_margin_mu.LowerBound;
fprintf('Robust Stability Margin (RS) = %.4f\n', RS_mu);

%% === Robust Performance Analysis ===
P_unc_mu = augw(G_unc, Wp, Wu);
CL_perf_mu = lft(P_unc_mu, K_mu_red);
[perf_margin_mu, perf_report_mu] = robustperf(CL_perf_mu);
disp('--- Robust Performance (μ) ---');
disp(perf_report_mu);
RP_mu = 1 / perf_margin_mu.LowerBound;
fprintf('Robust Performance Margin (RP) = %.4f\n', RP_mu);

%% Compare with H∞ results
fprintf('\nComparison with H∞ design:\n');
fprintf('H∞ RS margin: %.4f\n', RS);
fprintf('μ-synthesis RS margin: %.4f\n', RS_mu);
fprintf('H∞ RP margin: %.4f\n', RP);
fprintf('μ-synthesis RP margin: %.4f\n', RP_mu);

%% Simulate with 10 uncertainty samples
figure;
for i = 1:10
    G_sample = usample(G_unc);
    T_sample = feedback(G_sample * K_mu_red, eye(2));
    [y_sample, ~] = lsim(T_sample, r, t);
    
    subplot(2,1,1); hold on;
    plot(t, y_sample(:,1), 'LineWidth', 0.5);
    plot(t, y_sample(:,2), 'LineWidth', 0.5);
    
    % Compute control inputs
    e_sample = r - y_sample;
    u_sample = lsim(K_mu_red, e_sample, t);
    
    subplot(2,1,2); hold on;
    plot(t, u_sample(:,1), 'LineWidth', 0.5);
    plot(t, u_sample(:,2), 'LineWidth', 0.5);
end

subplot(2,1,1);
title('μ-Synthesis Response with 10 Uncertainty Samples (Angles)');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('Nominal Pitch','Nominal Yaw', 'Sample Pitches', 'Sample Yaws');
grid on;

subplot(2,1,2);
title('μ-Synthesis Control Inputs with 10 Uncertainty Samples');
xlabel('Time (s)'); ylabel('Voltage (V)');
legend('Nominal u1','Nominal u2', 'Sample u1s', 'Sample u2s');
grid on;