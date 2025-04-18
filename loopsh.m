s = tf('s');
wc = 5;
beta = 4;
alpha = 0.1;

W1 = (wc/s) * (s + beta*wc) / (s + wc) * (1 / (alpha*s + 1)) * eye(2);

[K, CL, gam] = loopsyn(G_nominal, W1);

wc = 5;            % 目标带宽
hf = 100;          % 两个高频极，保证可实现

G_inv = (1/(s/hf + 1)^2) * (G_unc \ eye(2));   % (1) 近似逆 + HF roll‑off
L_d   = (wc/s) * eye(2);                       % (2) 理想对角 wc/s 斜率
K_inv = minreal(tf(L_d * G_inv));              % (3) “预补偿”控制器
K_inv = ss(K_inv);                             %     转成 ss，方便 loopsyn

[Kinf,CLinf,gamma,info] = loopsyn(G_unc, G_unc*K_inv, 1);  % (4) α = 1
K = Kinf;         