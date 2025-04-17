clc; clear;close all
run('quanser_aero_parameters.m');
run('quanser_aero_state_space.m');

% Delta_1 = ultidyn('D_1', [1 1]);
% Delta_2 = ultidyn('D_2',[1 1]);

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

model = 'modle_build';  
blkK  = [model '/Controller'];  
blkD1 = [model '/Uncertain Plant/Delta_1'];
blkD2 = [model '/Uncertain Plant/Delta_2'];

set_param(blkK,'sys','K');

Nsamp   = 10;
tFinal  = 100;
allPitch = [];

Delta_1 = ultidyn('D_1',[1 1]);
Delta_2 = ultidyn('D_2',[1 1]);

for k = 1:Nsamp
    D1 = usample(Delta_1,1);
    D2 = usample(Delta_2,1);

    set_param(blkD1,'sys','D1');
    set_param(blkD2,'sys','D2');

    simOut = sim(model , ...
             'StopTime'   , num2str(tFinal) , ...
             'SolverType' , 'Fixed-step'    , ...
             'Solver'     , 'ode4'          , ...
             'FixedStep'  , '0.01'          , ...
             'SaveOutput' , 'off'           , ...
             'SignalLogging','on');
    pitchS = simOut.logsout.get('Pitch Position (rad)').Values;

    if k==1                               % allocate once
        t        = pitchS.Time;
        Nt       = numel(t);
        allPitch = zeros(Nt,Nsamp);
    end
    allPitch(:,k) = pitchS.Data;          % store kth trajectory
end

figure, hold on, grid on
plot(t , allPitch , 'b')                 % every column a blue line
xlabel('Time (s)'), ylabel('Pitch angle (rad)')
title(sprintf(['10 Monte‑Carlo runs  (ω_c = %.1f  δ = %d  α = %.1f)\\newline' ...
               'Weight order [4 4],  γ = %.2f'], wc,delta,alpha,gam))