# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : tools/pc_pid_tuner/v1_serial_2026-11-11/tests/serialtest.py
#  What    : PC-side test for PC PID tuner v1 (serial) (no board needed)
#  Date    : 2026-11-11
#  FUNCTIONS: w(), fmt(), pic(), wait()
# ==============================================================================================

# Linux/macOS only (uses os.openpty for a fake PIC32 serial port).
# Run:  QT_QPA_PLATFORM=offscreen python3 serialtest.py
import sys, os, time, threading, tty
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gui"))
import pid_tuner as pt
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg

# ---- fake PIC32 speaking the protocol on a pseudo-terminal ----
master, slave = os.openpty(); tty.setraw(slave)
slave_name = os.ttyname(slave)
sim = pt.SeesawSim(); live=[True]; lock=threading.Lock()
def w(line):
    data=(line+"\r\n").encode()
    while data:                      # pty buffer can fill during a run dump: retry
        try: data=data[os.write(master, data):]
        except BlockingIOError: time.sleep(0.001)
def fmt(t_ms, row): return f"D,{int(t_ms)},{row[1]:.2f},{row[2]:.2f},{row[3]:.0f},{row[4]:.1f},{row[5]:.1f},{row[6]:.1f}"
def pic():
    buf=b""; last=time.perf_counter(); acc=0
    os.set_blocking(master, False)
    while True:
        try: buf += os.read(master, 1024)
        except BlockingIOError: pass
        while b"\n" in buf:
            line, buf = buf.split(b"\n",1); f=line.decode().strip().split(",")
            with lock:
                if f[0]=="K": sim.kp,sim.ki,sim.kd = map(float,f[1:4]); w(f"A,{sim.kp:g},{sim.ki:g},{sim.kd:g}")
                elif f[0]=="S": sim.r=float(f[1])
                elif f[0]=="L": live[0]= f[1]=="1"
                elif f[0]=="X": sim.armed=False; w("KILL")
                elif f[0]=="R":
                    a,b,ms=float(f[1]),float(f[2]),int(f[3]); sim.reset(a); sim.r=a
                    for _ in range(250): sim.step()
                    rows=[]
                    for k in range(50+ms//4):
                        if k>=50: sim.r=b
                        rows.append(fmt((k-50)*4, sim.step()))
                    w("RUN_BEGIN"); [w(r) for r in rows]; w("RUN_END")
        now=time.perf_counter(); acc+=now-last; last=now
        n=int(acc/0.004); acc-=n*0.004
        with lock:
            for _ in range(n):
                row=sim.step()
                if live[0]: w(fmt(row[0]*1000,row))
        time.sleep(0.002)
threading.Thread(target=pic, daemon=True).start()
w("Flight controller start"); w("WHO_AM_I = 0x6C (LSM6DSO should be 0x6C)")

pg.setConfigOptions(antialias=False, background=pt.SURFACE, foreground=pt.TEXT2)
app = QtWidgets.QApplication(sys.argv); pt.apply_theme(app)
QtCore.QSettings("ELC275","PIDTuner").clear()
win = pt.MainWindow(); win.show()
def wait(sec):
    end=time.perf_counter()+sec
    while time.perf_counter()<end: app.processEvents(); time.sleep(0.005)
win.mode_combo.setCurrentIndex(1); wait(0.2)
win.port_combo.setCurrentText(slave_name); win.on_connect_clicked(); wait(1.5)
print("backend:", win.backend.describe(), "| rate:", win.rate_label.text(), "| live n:", win.live.n)
print("applied:", [g.applied.text()[-30:] for g in win.gains])
win.g_kp.set_value(4); win.g_kd.set_value(0.3); win.sp_spin.setValue(10); wait(0.5)
print("applied after change:", win.backend and [g._applied_value for g in win.gains])
win.from_spin.setValue(0); win.sp_spin.setValue(20); win.start_run(); wait(1.5)
for r in win.runs: print("run", r.num, r.kp, r.kd, len(r.array()), r.metrics)
win.raw_edit.setText("L,0"); win.send_raw(); wait(0.5); n1=win.live.n; wait(0.5); print("live stopped:", win.live.n==n1)
win.kill(); wait(0.3)
win.tabs.setCurrentIndex(2); wait(0.1); win.grab().save("shot_serial_console.png")
print("\n".join(win.console.toPlainText().splitlines()[:12]))
win.close()
