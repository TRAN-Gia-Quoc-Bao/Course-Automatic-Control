function [Rrob, Srob] = robuste(A, B, wd, Te)
%
% Function [Rrob, Srob]=robuste(A, B, wd, Te)
%
% computes a Robust Control (H-infinity).
%
% polynomial A : Denominator of the discret-time process model
% polynomial B : Numerator of the discret-time process model
% constant wd : is a disturbance frequency in (rad/s)
% constant Te : sample-time in seconds
%
% Created by: John J. Martinez, Grenoble-INP 2014.

G = tf(B, A, Te);
D = poly(exp([-0.1 + wd*i; -0.1 - wd*i]*Te));
W1 = tf(0.01, D, Te); %modèle perturbation
W2 = tf(2, 1, Te);%limitations des actionneurs
W3 = tf(6, 1, Te);%robustesse vis-à-vis les incertitudes

P = augw(G,W1,W2,W3);
K = tf(hinfsyn(P, 1, 1));
K = ss(K, 'min'); %numerical conditionning

[Rrob, Srob] = tfdata(K, 'v');