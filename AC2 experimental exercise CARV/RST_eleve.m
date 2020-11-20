%Initialisation
clear all: close all; clc

Tfin=200;
X_scope =100;

Te=1/800; %Te=0.0013 seconds;
Ts=Te;

%% Identification


%% Commande RST par placement de pôles
wd = 450; %pulsation de la perturbation

R=0;
S=1;

%% Commande Robuste 
%utiliser la function robuste : 
%[Rrob,Srob]=robuste(A,B,wd,Te)

Rrob=0;
Srob=1;