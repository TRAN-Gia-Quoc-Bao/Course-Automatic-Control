%% Initialisation TP Bacs, 
% Ense3 GRENOBLE-INP 

clear all
close all
clc

% paramétres du système
a=25*10^(-2);
b=34.5*10^(-2);
c=10*10^(-2);
w=3.5*10^(-2);
R=36.4*10^(-2);
L=35*10^(-2);
H1max=35*10^(-2);
H2max=35*10^(-2);
H3max=35*10^(-2);

% contrainte sur la commande
Qmax=1.4e-4; %m^3/s 

% paramétres alpha_i et C_i (Débit sortant = C_i*H_i^(alpha_i))


% C1=2.0338e-04; %Pour H autour de 15cm (MAQUETTE 1)
% C2=2.0309e-04;
% C3=1.9725e-04;
% Alfa1=0.3325; 
% Alfa2=0.3135; 
% Alfa3=0.2975;

C1=2.3314e-04; %Pour H autour de 15cm (MAQUETTE 2)
C2=2.7312e-04;
C3=2.4837e-04;
Alfa1=0.3068; 
Alfa2=0.4120; 
Alfa3=0.3412;


% C1=1.7039e-04; %Pour H autour de 15cm (MAQUETTE 3)
% C2=1.9936e-04;
% C3=1.8706e-04;
% Alfa1=0.2915; 
% Alfa2=0.3997; 
% Alfa3=0.3445;


Alfa=[Alfa1 Alfa2 Alfa3];
CC=[C1 C2 C3];
 
% Niveaux à l'équilibre (point de fonctionnement)
H30=0.15;
Q0= C3*H30^Alfa3 % à compléter (expression littérale en fonction de H30)
H10= (Q0/C1)^(1/Alfa1)  % à compléter (expression littérale en fonction de Q0)
H20= (Q0/C2)^(1/Alfa2) % à compléter (expression littérale en fonction de Q0)


%%% Point de fonctionnement:
H0=[H10;H20;H30]

%%
% Sections transversalles aux point de fonctionnement
S1=a*w;
S2=w*(c+H20/L*b);
S3=w*sqrt(R^2-(R-H30)^2);


%% à completer
% Les matrices du systéme linéarisé
A=[-(C1*Alfa1*H10^(Alfa1-1))/S1  0  0
    (C1*Alfa1*H10^(Alfa1-1))/S2  -(C2*Alfa2*H20^(Alfa2-1))/S2 0
    0  (C2*Alfa2*H20^(Alfa2-1))/S3  -(C3*Alfa3*H30^(Alfa3-1))/S3]; 
B=[1/S1 0 0]';
C=[0 0 1];
D=[0];

sys = ss(A,B,C,D);
Ctr = ctrb(sys);
rgCtr = rank(Ctr);
Obs = obsv(sys); 
rgObs = rank(Obs);

Pole = pole(sys);
PoleL1 = 10*Pole;
L1=place(A',C',PoleL1)';
H0L =[0.1 ; 0.1 ; 0.1];
PoleL2 = 5*Pole;
L2=place(A',C',PoleL2)';
PoleL3 = 20*Pole;
L3=place(A',C',PoleL3)';
%% Tracé des points d'équilibres possible en boucle ouverte
% Qe = (0:(1.4e-5)/1000:1.4e-4);
% H1 = (Qe./C1).^(1/Alfa1);
% H2 = ((C1.*H1.^(Alfa1))./C2).^(1/Alfa2);
% H3 = ((C2.*H2.^(Alfa2))./C3).^(1/Alfa3);
% figure();
% plot(Qe,H1,Qe,H2,Qe,H3);
% grid on;
% title("Points d'équilibre possibles en boucle ouverte");
% xlabel("Débit d'entré de H1 ( m^3/s )");
% ylabel("Hauteur ( m)");
% legend("H1","H2","H3");
%% Tracé des résultats de l'obseravteur
figure();
plot(Estimated_level.time,Estimated_level.signals(1).values,Estimated_level.time,Estimated_level.signals(2).values);
grid on;
title("Implementation de l'observateur pour H0L=[ 0.1 ; 0.1 ; 0.1] avec une perturbation sur la mesure en échelon de 0.05m");
xlabel("Temps (s)");
ylabel("Hauteur (m)");
legend("H1","H2","H3","H1 estimated","H2 estimated","H3 estimated");
figure();
plot(Error.time,Error.signals.values);
grid on;
title("Erreur de l'observateur pour H0L=[ 0.1 ; 0.1 ; 0.1]avec une perturbation sur la mesure en échelon de 0.05m");
xlabel("Temps (s)");
ylabel("Hauteur (m)");
legend("Erreur H1","Erreur H2","Erreur H3");
%% Tracé des résultats de l'e^perience
figure();
plot(Levels_real.time,Levels_real.signals(1).values,Levels_real.time,Levels_real.signals(2).values,Levels_real.time,Levels_real.signals(3).values,Levels_real.time,Levels_real.signals(4).values);
grid on;
title("Implementation de l'observateur pour H0L=[ 0 ; 0 ; 0] sur le système réel");
xlabel("Temps (s)");
ylabel("Hauteur (m)");
legend("Commande","H1","H2","H3","H1 estimated","H2 estimated","H3 estimated");
figure();
plot(Error_real.time,Error_real.signals.values);
grid on;
title("Erreur de l'observateur pour H0L=[ 0 ; 0 ; 0] sur le système réel");
xlabel("Temps (s)");
ylabel("Hauteur (m)");
legend("Erreur H1","Erreur H2","Erreur H3");
figure();
plot(Levels_real.time,Levels_real.signals(1).values);
grid on;
title("Implementation de l'observateur pour H0L=[ 0 ; 0 ; 0] sur le système réel");
xlabel("Temps (s)");
ylabel("Débit(m^3/s)");
legend("Commande");