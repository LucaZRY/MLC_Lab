s = tf('s');
wc = 5;
beta = 4;
alpha = 0.1;

W1 = (wc/s) * (s + beta*wc) / (s + wc) * (1 / (alpha*s + 1)) * eye(2);

[K, CL, gam] = loopsyn(G_nominal, W1);