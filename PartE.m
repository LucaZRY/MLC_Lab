% File: partE_mu_synthesis.m

% Load Parameters and Plant
run('quanser_aero_parameters.m');
run('quanser_aero_state_space.m');

% Build Nominal and Uncertain Plant
G_unc = ss(A, B, C, D);       % uncertain plant
G_nom = G_unc.NominalValue;  % nominal plant

% Sample-based Uncertainty Fitting
G_samples = usample(G_unc, 300);
[~, info] = ucover(G_samples, G_nom, 4, [], 'InputMult');
W_I = info.W1;                      % multiplicative uncertainty weight

W_I_Pitch = W_I;
W_I_Yaw   = W_I;

% Define weighting functions (from Part D)
s = tf('s');
Wp = ss(blkdiag(1000/(s + 0.33), 1000/(s + 0.33)));
Wu = ss(blkdiag(0.1, 0.1));

% Define Uncertainty (Input Multiplicative)
Delta = ultidyn('Delta', [2 2]);

% Connect Uncertainty into G_unc
I = eye(2);
Delta_blk = feedback(I, W_I * Delta);  % input multiplicative uncertainty
G_pert = G_nom * Delta_blk;

% Define I/O Names
G_pert.InputName = {'u1','u2'};
G_pert.OutputName = {'y1','y2'};
Wp.InputName = {'y1','y2'};
Wp.OutputName = {'ep1','ep2'};
Wu.InputName = {'u1','u2'};
Wu.OutputName = {'eu1','eu2'};

% Connect everything for generalized plant
Sum1 = sumblk('u1 = r1 - u1c');
Sum2 = sumblk('u2 = r2 - u2c');
Sum3 = sumblk('y1m = y1');
Sum4 = sumblk('y2m = y2');

G_all = connect(G_pert, Wp, Wu, Sum1, Sum2, Sum3, Sum4, ...
    {'r1','r2','u1c','u2c'}, {'ep1','ep2','eu1','eu2','y1m','y2m'});

% Prepare for musyn
nmeas = 2; % measurements (y1m, y2m)
ncon  = 2; % controls (u1c, u2c)
[K_mu, CL_mu, info_mu] = musyn(G_all, nmeas, ncon);

% Optional: reduce order to match Hinf
K_mu = balred(K_mu, order(K_mu));

% Save for usage in other scripts
save('K_mu.mat', 'K_mu');

% Bode and Step Plots
figure;
bode(K_mu), grid on, title('Bode plot of \mu-synthesis controller (musyn)');

% Simulate step response from ref to outputs (2 outputs, so need input channels)
t = 0:0.01:10;
r = ones(length(t), 2); % 2-channel unit step input
y = lsim(CL_mu, r, t);

figure;
plot(t, y(:,1), t, y(:,2));
grid on;
title('Closed-loop Step Response with \mu-controller (musyn)');
xlabel('Time (s)'); ylabel('Output');
legend('Pitch','Yaw');
