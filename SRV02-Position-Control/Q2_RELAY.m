%constants for relay tuning model
tau=0.0254;
K=1.5286;

%y, u, t derived from relay simulation
out = sim("RELAY_TUNING");
t = get(out,"tout");
y = get(out,"y");
u = get(out,"u");

%parameters for pid tuning
d= max(u);     %square wave ampl
a= max(y);     %output ampl
[P,T]=findpeaks(y);
Tc=t(T(6))-t(T(5));

Kc=4*d/(a*pi);   %critical gain
wc=2*pi/Tc  ;    %crit freq

kp=0.6*Kc
ki=0.5*Tc
kv=0.125*Tc
