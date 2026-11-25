# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : tools/pc_pid_tuner/v1_serial_2026-11-11/tests/simtest.py
#  What    : PC-side test for PC PID tuner v1 (serial) (no board needed)
#  Date    : 2026-11-11
#  FUNCTIONS: pid(), run()
# ==============================================================================================

import numpy as np, random
DT=0.004
def pid(e, edot, Kp, Ki, Kd, eint, eintmax, umax):
    eint += e*DT
    eint = max(-eintmax, min(eintmax, eint))
    u = Kp*e + Ki*eint + Kd*edot
    u = max(-umax, min(umax, u))
    return u, eint
def run(Kp,Ki,Kd,r=15,y0=0,T=4,G=12,B=4,TAU=0.05,imb=10,an=0.15,rn=3,sub=4):
    th=y0; om=0; m=1500; eint=0; out=[]
    for k in range(int(T/DT)):
        s=th+random.gauss(0,an); rate=om+random.gauss(0,rn)
        e=r-s; edot=-rate
        u,eint=pid(e,edot,Kp,Ki,Kd,eint,20,500)
        cmd=min(2000,max(1000,1500+u))
        h=DT/sub
        for _ in range(sub):
            m+=(cmd-m)/TAU*h
            a=G*(m-1500-imb)-B*om
            om+=a*h; th+=om*h
            if th>40: th=40; om=min(om,0)
            if th<-40: th=-40; om=max(om,0)
        out.append((k*DT,th,cmd))
    a=np.array(out); y=a[:,1]
    n=(y-y0)/(r-y0)
    tr = a[np.argmax(n>=0.9),0] if (n>=0.9).any() else None
    os_=max(0,(n.max()-1)*100)
    sat=np.mean((a[:,2]<=1000)|(a[:,2]>=2000))*100
    ss=np.mean(r-y[-100:]); pp=y[-250:].max()-y[-250:].min()
    return f"Kp={Kp:5} Ki={Ki:4} Kd={Kd:4} | t90={tr} OS={os_:5.1f}% ss_err={ss:6.2f} last1s p-p={pp:5.1f} sat={sat:4.1f}%"
random.seed(1)
for kp in [0.5,1,2,3,5,7,9,12]: print(run(kp,0,0))
for kd in [0.1,0.2,0.4,0.6,1.0,2.0,4.0]: print(run(5,0,kd))
for ki in [1,3,6,10]: print(run(5,ki,0.4))
for kp,kd in [(10,0.6),(15,0.8),(20,1.0)]: print(run(kp,0,kd))
