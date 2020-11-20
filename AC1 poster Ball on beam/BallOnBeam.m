%% Poster: Ball on beam

%% Default commands
close all; clear all; clc;

% Parameters
% ----------
J     = 4.6E-5;
f     = 7E-5; 
rm    = 0.33;
lm    = 0.001;
imax  = 100;
nm    = 960;
k     = 0.026; % k = Phi 

k1    = 7;
k2    = 28.65;
k3    = 0.0159;
ki    = 1.6; 
kb    = 6.1;                                        
Te    = 0.2;

trdes = 5;
taudes = trdes/4.75;

% Current control of motor
% ------------------------
trdesi = 1.5*1e-3;
taudesi = trdesi/3;
tii   = 0.5*lm/rm; % 0.003;
kpi   = rm*tii*ki/taudesi; %3.2;

% Control of angle T
% ------------------

tff = 0.025;   % filtrage de la consigne d'angle
xides = 0.7;
trdes = 0.1;
wndes = 0.43*2*pi/trdes; 
kp = nm*J*wndes^2/(k2*k*ki); %27
kv = (2*xides*J*wndes - f)/(kp*ki*k*k3);%kv = 0.0934;

% Controlling the system using PID
% --------------------------------

% Définition des paramètres du modèle 
trdes2 = 5;
taudes2 = trdes2/4.75;
a1 = -2;
a2 = 1;
b1 = k1*kb*Te*Te/(2*k2);
b2 = k1*kb*Te*Te/(2*k2);

% Définition des performances désirées
a = exp(-Te/taudes2);
p1 = -2*exp(-Te/taudes2);
p2 = exp(-2*Te/taudes2);
p3 = 0;
p4 = 0;

% Solve Bezout equation by hand
ap1 = a1-1;
ap2 = a2-a1;
ap3 = -a2;

Pdes = [1 p1 p2 p3 p4];
x = inv([1 0 0 0 0; ap1 1 b1 0 0; ap2 ap1 b2 b1 0; ap3 ap2 0 b2 b1; 0 ap3 0 0 b2])*Pdes';
% Controller parameters
s1 = x(2);
r0 = x(3);
r1 = x(4);
r2 = x(5);

% Controlling the system using PID RST
% ------------------------------------
T = r0 + r1 + r2;

% Controlling the system using state-feedback with integral action
% ----------------------------------------------------------------

% Observer
A = [0 1; 0 0];
B = [0; kb/k2];
C = [k1 0];
D = 0;
[Ad, Bd, Cd, Dd] = ssdata(c2d(ss(A, B, C, D), Te));
Ld = acker(Ad', Cd', [0.3, 0.3])';
pole(ss(Ad - Ld*Cd, Bd, Cd, Dd));

% Integral action
Ai = 1;
Bi = 1;
Ci = 1;
Di = 0;

Ae = [Ad zeros(2, 1); -Cd 1];
Be = [Bd; 0];
Ce = [Cd 0];
De = 0;

rank(ctrb(ss(Ae, Be, Ce, De)));

Fe = acker(Ae, Be, [exp(-Te/taudes), exp(-Te/taudes), exp(-Te/taudes)/20]);
pole(ss(Ae - Be*Fe, Be, Ce, De));