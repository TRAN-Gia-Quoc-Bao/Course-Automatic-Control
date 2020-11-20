% Model of the q1 position system
Te=Ts;

A2=[0 1; 0 0];
B2=[0 1/Jd]';
C2=[1 0];
%observer design
    P2=[-5 -5.1]; %10*[-20   -21   -10   -50   -50]
    H2=(place(A2',C2',P2))';
    
Aobs2=A2-H2*C2;
Bobs2=[B2 H2];
Cobs2=[0 1]; %recover just the speed w1=dot{q1}
Dobs2=zeros(1,2);

Obs2=ss(Aobs2,Bobs2,Cobs2,Dobs2);