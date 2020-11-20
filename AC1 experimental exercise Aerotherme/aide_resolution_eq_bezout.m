clear all,
close all
clc

%%
% D�finition des param�tres du mod�le

Te = 2.0;
U0 = 5;
V0 = 6;
k1=0.78;
% k1 = 0.64;
k2=-0.25;
Ts = 0.1;
Te=2;
tau=7.64;
% tau = 6.32;
tau_des=3;
b1 = k1*(1 - (1 + Te/tau)*exp(-Te/tau));
b2 = k1*(Te/tau - 1 + exp(-Te/tau))*exp(-Te/tau);
a1 = -2*exp(-Te/tau);
a2 = exp(-2*Te/tau);
B = [b1 b2];
A = [1 a1 a2];

%% 
% D�finition des performances d�sir�es 
p1=-2*exp(-Te/tau_des);
p2=exp(-2*Te/tau_des);
p3=0;
p4=0;
%%
% R�solution manuelle de l'�quation de Bezout
ap1=a1-1;
ap2=a2-a1;
ap3=-a2;

Pdes=[1 p1 p2 p3 p4];
x=inv([1 0 0 0 0; ap1 1 b1 0 0; ap2 ap1 b2 b1 0;ap3 ap2 0 b2 b1;...
   0 ap3 0 0 b2])*Pdes';
% param�tres du correcteur
sp1=x(2); % il s'agit de s'1 du cours.
r0=x(3);
r1=x(4);
r2=x(5);

R = [ r0 r1 r2];
S = [ 1 sp1-1 -sp1];
T = [ r0+r1+r2 ];
conv(A,S)
conv(B,R)
[conv(A,S)] + [0 conv(B,R)]
zplane(conv(T,R),[conv(A,S)] + [0 conv(B,R)])
title("Diagramme de p�les et z�ros");

%%

plot(X.time,X.signals.values)
grid on;
xlabel("time(s)");
ylabel("tension(V)");
title("R�ponse unitaire et rejet de perturbation avec correcteur RST");
legend("Consigne","Commande","Sortie","Perturbation");