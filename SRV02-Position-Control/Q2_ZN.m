s = tf('s');
sys = K/(tau*s+1);

[y,tOut] = step(sys);
...dt=diff(tOut);
dt=0.001;
...y=theta_l;
dy=diff(y)./diff(tOut);
dydy=diff(dy)./diff(tOut(2:end));
[DP,IndexP]=max(dy);
theta0=y(IndexP);
t0=IndexP*dt;

a=-(-DP*t0+theta0);
ztau=t0-theta0/DP;

kp=1.2/a;  
ki=2*ztau;
kv=0.5*ztau;
