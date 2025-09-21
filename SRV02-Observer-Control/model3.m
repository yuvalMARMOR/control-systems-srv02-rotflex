שCsys=ss(A,B,C,D) %define system
Ts=0.002;       %sampling time
Dsys=c2d(Csys,Ts);  %convert to discrete
[AD,BD,CD,DD]=ssdata(Dsys);%retrieve matrices

zeta=0.6;
wn=20;

sig=zeta*wn;
wd=wn*sqrt(1-zeta^2);
s3=20;
s4=25;

pCont=[-sig+wd*j,-sig-wd*j,-s3,-s4];%poles in s-domain
pDisc=exp(pCont.*Ts)     %poles in z-domain
cont=place(A,B,pCont);
K=place(AD,BD,pDisc)

%s=tf('s')
%systf=C*(s*eye-A)^(-1)*B
