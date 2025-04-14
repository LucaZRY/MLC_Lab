clc; clear;
run('quanser_aero_parameters.m');
run('quanser_aero_state_space.m');

D_1 = ultidyn('D_1',1);
D_2 = ultidyn('D_2',1);

Delta_1 = usample(D_1,1);  
Delta_2 = usample(D_2,1);

G_unc = minreal(ss(A, B, C, D)); 
G_nominal = G_unc.NominalValue;
G_samples = usample(G_unc, 300);

[~,info] = ucover(G_samples, G_nominal,[2, 2],[],'InputMult');

W_I = info.W1; 
W_I_Pitch = W_I(1,1);
W_I_Yaw = W_I(2,2);

G_verify = usample(G_unc, 300);

s = tf('s');
K = [(1711*s+4910)/(s+50) -(1557*s+5153)/(s+50);
     (2432*s+7817)/(s+50) (921.5*s+3308)/(s+50)];
