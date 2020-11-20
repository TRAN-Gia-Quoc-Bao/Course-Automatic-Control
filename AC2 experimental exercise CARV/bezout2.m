function [Ro,So]=bezout2(A,B,Hs,Hr,P)
%
%function [Ro,So]=bezout2(A,B,Hs,Hr,P)
%
%Solves the Bezout equation AHsSo+BHrRo=P 
%for given B, A, Hs, Hr, P polynomials.
%Delay and discretization delay need to be integrated in B.
%
%inputs:
%A=[a0 a1 ... aNa] ... vector of model denominator coefficients A=a0 + a1z^(-1) + a2z^(-2) +...+ aNaz^(-Na) 
%B=[b0 b1 ... bNb] ... vector of model numerator coefficients B=b0 + b1z^(-1) + b2z^(-2) +...+ bNbz^(-Nb)
%Hs=[hs0 hs1 ... hsNhs] ... vector of controller denominator fixed part Hs=hs0 + hs1z^(-1) +...+ hsNhsz^(-Nhs) 
%Hr=[hr0 hr1 ... hrNhr] ... vector of controller denominator fixed part Hr=hr0 + hr1z^(-1) +...+ hrNhrz^(-Nhr) 
%P=[p0 p1 ... pNp] ... vector of desired polynomial coefficients P=p0 + p1z^(-1) + p2z^(-2) +...+ pNpz^(-Np)
%
%outputs:
%Ro=[rp0 rp1 rp2 ...] ... vector of coefficients for resulted controller numerator 
%So=[sp0 sp1 sp2 ...] ... vector of coefficients for resulted controller denominator 
%
%written by: John J. Martinez-Molina (ENSE3, GRENOBLE-INP)
%11th december 2012

na=length(A)-1;
nb=length(B)-1;
np=length(P)-1;
ns=length(Hs)-1;
nr=length(Hr)-1;


%%
Ah=conv(A,Hs); 
Bh=conv(B,Hr);

nah=length(Ah)-1;
nbh=length(Bh)-1;


%%
if nah<nbh,
    Ah=[Ah zeros(1,nbh-nah)];
elseif nbh<nah
    Bh=[Bh zeros(1,nah-nbh)];
end
    nah=length(Ah)-1;
    nbh=length(Bh)-1;

if (np>nah+nbh-1), 
    %disp('Bezout Equation Error: too many poles for polynomial P')
    %disp('Order of polynomials A and B are too low! Add a polynomial of higher order for Hs or Hr.');
    Ah=[Ah zeros(1,ceil(0.5*np)+1)]
    Bh=[Bh zeros(1,ceil(0.5*np)+1)]
    nah=length(Ah)-1;
    nbh=length(Bh)-1;
end;

% increase size of P using zeros if necessary
if (np<nah+nbh-1),
 	P=[P zeros(1,nah+nbh-1-np)];  
end;
P=P';

% Computation of matrix M
M=[];

for j=1:nah,
V=[ zeros(j-1,1); Ah'; zeros(nah-j,1) ];
M=[M V];
end

for j=1:nbh,
V=[ zeros(j-1,1); Bh'; zeros(nbh-j,1) ];
M=[M V];
end

% Compute vector X 
X=M\P; %inv(M)*P

% make X row vector
nsp=nbh-1;
nrp=nah-1;

X=X';

So=X(1:nsp+1);
%So=So(1:length(B)-1); %just to recover suitable values of So
Ro=X(nsp+2:nsp+nrp+2);




