%% Practical work: Aerotherme

%% Default commands
clear all; close all; clc;

%% Parameters
Ts = 0.1;
Te = 2.0;
U0 = 5;
V0 = 6;
  
k1 = 0.78;
% k1 = 0.64;
k2 = -0.25;
tau = 7.64;
% tau = 6.32;
tau_des = 3;
 
%figure(1);
%subplot(211);
%step([k1],[tau^2 2*tau 1]);
%subplot(212);
%step([k2],[tau^2 2*tau 1]);
F= tf([k1],[tau^2 2*tau 1]);
Fd=c2d(F,Te);
b1 = k1*(1 - (1 + Te/tau)*exp(-Te/tau))
b2 = k1*(Te/tau - 1 + exp(-Te/tau))*exp(-Te/tau)
a1 = -2*exp(-Te/tau)
a2 = exp(-2*Te/tau)
p1_des = -2*exp(-Te/tau_des);
p2_des = exp(-2*Te/tau_des);
B = [b1 b2];
A = [1 a1 a2];

%% Pole compensation
A1 = [b1 1; b2 -1];
B1 = [p1_des + 1; p2_des];
coeff = inv(A1)*B1;
r0 = coeff(1);
s = coeff(2);
r1 = a1*r0;
r2 = a2*r0;

R = [r0 r1 r2];
S = [1 s-1 -s];
% T = [R];
T = [r0+r1+r2];

%Hbo = tf([conv(B,R)],[conv(A,S)],Te);
zplane([b1*r0 b2*r0],[1 s-1+b1*r0 b2*r0-s]);
title("Pole-zero diagram");

%% Plot
% plot(X.time,X.signals.values);
% grid on;
% xlabel("Time (s)");
% ylabel("Voltage (V)");
% title("Step response with pole compensation control");
% legend("Reference", "Control", "Output", "Disturbance");
%%
%A = [0.026 0 0 1; 0 0.026 9.15e-3 -1; 0 9.15e-3 0.026 0; 0 0 9.15e-3 0];
%coef = inv(A)*B;
%r0 = coef(1);
%r1 = coef(2);
%r2 = coef(3);
%s = coef(4);