close all; clear; clc;

quanser_aero_parameters;
quanser_aero_state_space;

G_unc = ss(A, B, C, D);  

wc = 5;
M = 3;
A = 1000;

Wu = tf(1/10) * eye(2);

Wp_single = makeweight(A, wc, 1/M);
Wp = blkdiag(Wp_single, Wp_single);

P = augw(G_unc, Wp, Wu);

[K_mu, CL, mu_syn] = musyn(P, 2, 2);

% 1. Create closed-loop system for uncertain plant
CL_unc = lft(P, K_mu);  % closed-loop system with uncertainty

% 2. Check Robust Stability
[stabmarg, destabunc, report_stab] = robuststab(CL_unc);
disp('Robust Stability Margin:');
disp(stabmarg.LowerBound);

% 3. Check Robust Performance
[perfmarg, destabuncP, report_perf] = robustperf(CL_unc);
disp('Robust Performance Margin:');
disp(perfmarg.LowerBound);

% 4. Simulate for 10 uncertain samples
samples = usample(CL_unc, 10);  % 10 random samples

% 5. Create time vector and reference input
t = 0:0.01:5;
r = [ones(size(t)); zeros(size(t))];  % step input in 1st channel

% 6. Plot step responses for each sample
figure;
hold on;
for i = 1:10
    y = lsim(samples(:,:,i), repmat(r', 1, 1), t);
    plot(t, y(:,1), 'b');  % plot pitch response
end
title('Step Response for Pitch (10 Samples)');
xlabel('Time (s)');
ylabel('Response');
grid on;
hold off;

figure;
hold on;
for i = 1:10
    y = lsim(samples(:,:,i), repmat(r', 1, 1), t);
    plot(t, y(:,2), 'r');  % plot yaw response
end
title('Step Response for Yaw (10 Samples)');
xlabel('Time (s)');
ylabel('Response');
grid on;
hold off;
