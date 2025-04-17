clc; clear;close all
run('quanser_aero_parameters.m');
run('quanser_aero_state_space.m');

Delta_1 = ultidyn('D_1', [1 1]);
Delta_2 = ultidyn('D_2',[1 1]);

G_unc = minreal(ss(A, B, C, D)); 
G_nominal = G_unc.NominalValue;
G_samples = usample(G_unc,300);

[~, info] = ucover(G_samples, G_nominal, [4,4], [], 'InputMult');
W_I = info.W1;

W_I_Pitch = W_I(1,1); 
W_I_Yaw   = W_I(2,2);

s  = tf('s');
wc = 5;
delta = 8;
alpha = 0.6;

z = wc;  
p = wc/delta;
W1 = (wc/s) * (s/z+1)/(s/p+1) * eye(2);

[K,CL,gam,info] = loopsyn(G_nominal , W1);
fprintf('γ (robustness margin) = %.3f\n',gam)

% get_param('modle_build/Controller','MaskNames')