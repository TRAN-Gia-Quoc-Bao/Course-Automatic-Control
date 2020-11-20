%% Practical work: Gyroscope

%% Default commands
clear all; close all; clc;

%% Parameters
Ka = 0.067; Ib = 0.012; Jb = 0.018; Kb = 0.030; Ic = 0.0092; 
Jc = 0.023; Kc = 0.022; Id = 0.015; Jd = 0.027;
Ts = 0.02; % sampling time for simulation
% rup = 8000;
% rdo = -8000;
% maxdif = 24400*825*Ts;
% %--------
% %Proportional controller w1 (Omega)
% Kp = 0.2; 
% Te = Ts; % used into some of  simulink blocks
% speed_w1_estimation  % run this to obtain w1_estimator parameters
% %--------
% % Damping compensation secs
% C0 = 0; %Nm %Attention : 0.070 Nm pour Gyro_1  et 0.051 Nm pour Gyro_2
%--------
Omega = 42; %rd/s (~400rpm)

%% State space model of the Gyroscope
A = [0                    0                   0                   1                   0                   ;
     0                    0                   0                   0                   1                   ;
     0                    0                   0                   0          Jd*Omega/(Ic + Id)           ;
     0                    0                   0                   0                   0                   ;
     0                    0      -Jd*Omega/(Id + Ka + Kb + Kc)    0                   0                   ];
B = [0                0           ;
     0                0           ;
     0                1/(Ic + Id) ;
     -1/(Jb + Jc)     0           ;
     0               0];
C = [1    0   0   0   0;
     0    1   0   0   0];
D = zeros(2, 2);
model = ss(A, B, C, D, 'Statename', {'theta3', 'theta4', 'w2', 'w3', 'w4'}, 'Inputname', {'T1', 'T2'}, 'Outputname', {'theta3', 'theta4'});

%% Stability analysis
p = pole(model); % not stable
z = zero(model); % the last zero is not stable

%% Frequency domain analysis
figure();
bodemag(model);
grid on;
title('Frequency analysis of the open loop system');

%% Temporary response
figure();
initial(model, [0 0 0 0 1]);
grid on;
title('Free response of the open loop system');

%% Controllability and Observability tests
Ob = obsv(model);%....
rOb = rank(Ob); % 5, the sys is observable 
Co = ctrb(model);
rCo = rank(Co); % 5, the sys is controllable

%% Control
Q = C'*C;
% R=[4 0; 0 0.015];
R = [0.5 0;
     0 0.04];
K = lqr(A, B, Q, R);
g = inv(C*inv(-A + B*K)*B) ;% static gain
% Computing the max control output
xmax = pi*20*[1; 1; 0; 0; 0]/180;
umax = abs(K*xmax); % verify the constraints on the control signal
sysBF = ss(A - B*K, B*g, C, D, 'Statename',{'theta3','theta4', 'w2', 'w3', 'w4'}, 'Inputname', {'T1', 'T2'}, 'Outputname', {'theta3', 'theta4'});
p_lqr = pole(sysBF)'; % closed-loop poles

% Test the step response
% figure();
% step(sysBF);
% grid on;
% title('The unit step response given by the state-feedback controller');

%% Kalman filter gain
W = B*B'; 
V = 0.01*eye(2, 2); %test: 0.1 and 0.01
L = lqr(A', C', W, V)';
Aobs = A - L*C;
Bobs = [B L];
Cobs = eye(5);
Dobs = zeros(5, 4);
Obs = ss(Aobs, Bobs, Cobs, Dobs, 'Statename', {'err-theta3', 'err-theta4', 'err-w2', 'err-w3', 'err-w4'}, 'Inputname', {'T1', 'T2', 'theta3', 'theta4'}, 'Outputname', {'err-theta3', 'err-theta4', 'err-w2', 'err-w3', 'err-w4'});
x_est_initial = [0 0 0 0 1];%

figure()
initial(Obs, x_est_initial);

%% ROBUSTNESS ANALYSIS
% Uncertain system (omega 25%)
omega_unc = ureal('omega_unc', 42, 'Range', [30, 52]);
% Uncertain matrices (note that only A in the real system changes; the matrix A into the observer does not change)
A_unc = [0 0 0 1 0; 0 0 0 0 1; 0 0 0 0 Jd*omega_unc/(Ic + Id); 0 0 0 0 0; 0 0 -Jd*omega_unc/(Id + Ka + Kb + Kc) 0 0];
B_unc = B;
C_unc = C;
D_unc = D;
model_unc = ss(A_unc, B_unc, C_unc, D_unc);
% Uncertain closed-loop system 
sysBFunc = ss([A_unc  -B*K; L*C   A - B*K - L*C], [B*g; B*g], [C_unc zeros(2, 5)], zeros(2, 2));
% Plot
figure();
step(sysBFunc);
grid on;

figure();
bodemag(sysBFunc);
grid on;

%% Integral Action
Ae = [A zeros(5, 2); -C zeros(2, 2)];
Be = [B; zeros(2, 2)];
Qe = zeros(7, 7);
Qe(6, 6) = 1;
Qe(7, 7) = 1;
Re = R/1000;
Ke = lqr(Ae, Be, Qe, Re);
F = Ke(:, 1 : 5);
H = Ke(:, 6 : 7);
%max expected control output u
umaxInt = abs(Ke*[xmax; 0; 0]);

%%
%Simulation sysBF
sysBFint = ss(Ae - Be*Ke, [zeros(5, 2); eye(2)], [C zeros(2, 2)], zeros(2, 2), 'Statename', {'theta3', 'theta4', 'w2', 'w3', 'w4', 'z1', 'z2'}, 'Inputname', {'r1', 'r2'}, 'Outputname', {'theta3', 'theta4'}); 

figure();
step(sysBFint);
grid on;
title('Step response of the observed-state feedback controller with integral action');

%% Robust
% Ae_unc = [A_unc zeros(5, 2); -C_unc zeros(2, 2)];
% Be_unc = [B_unc; zeros(2, 2)];
% Ce_unc = [C_unc zeros(2, 2)];
% De_unc = zeros(2, 2);
% Uncertain system
% modele_unc = ss(Ae_unc - Be_unc*Ke, [zeros(5, 2); eye(2)], [C_unc zeros(2, 2)], zeros(2, 2), 'Statename', {'theta3', 'theta4', 'w2', 'w3', 'w4', 'z1', 'z2'}, 'Inputname', {'r1', 'r2'}, 'Outputname', {'theta3', 'theta4'});
% figure();
% step(modele_unc);
% grid on;

% Uncertain close-loop system 
syseBFunc = ss([A_unc  -B_unc*F -B_unc*H; L*C_unc  A_unc - L*C_unc - B_unc*F -B_unc*H; -C_unc zeros(2,7)], [zeros(10, 2); eye(2)], [C_unc zeros(2, 7)], zeros(2, 2));
% Plot
figure();
bodemag(syseBFunc);
grid on;

figure();
step(syseBFunc);
grid on;