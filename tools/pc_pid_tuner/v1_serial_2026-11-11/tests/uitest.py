# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : tools/pc_pid_tuner/v1_serial_2026-11-11/tests/uitest.py
#  What    : PC-side test for PC PID tuner v1 (serial) (no board needed)
#  Date    : 2026-11-11
#  FUNCTIONS: wait(), run(), counted()
# ==============================================================================================

# NOTE: clears the saved GUI settings (QSettings "ELC275/PIDTuner") before running.
# Run headless:  QT_QPA_PLATFORM=offscreen python3 uitest.py   (saves shot_*.png here)
import sys, time
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gui"))
import pid_tuner as pt
from PySide6 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg
pg.setConfigOptions(antialias=False, background=pt.SURFACE, foreground=pt.TEXT2)
app = QtWidgets.QApplication(sys.argv)
pt.apply_theme(app)
QtCore.QSettings("ELC275","PIDTuner").clear()
w = pt.MainWindow(); w.resize(1440, 900); w.show()
def wait(sec):
    end = time.perf_counter()+sec
    while time.perf_counter() < end:
        app.processEvents(); time.sleep(0.005)
wait(1.0)
def run(kp, ki, kd, dur=3.0):
    w.g_kp.set_value(kp); w.g_ki.set_value(ki); w.g_kd.set_value(kd)
    w.from_spin.setValue(0); w.sp_spin.setValue(15); w.dur_spin.setValue(dur)
    wait(0.1); w.start_run(); wait(dur+0.6)
for g in [(1,0,0),(3,0,0),(6,0,0),(6,0,0.4),(6,3,0.4)]:
    run(*g)
for r in w.runs: print(r.num, r.kp, r.ki, r.kd, r.metrics, len(r.data))
# hover over runs plot
sc = w.rs.widget.scene()
p = w.rs.angle; vb = p.vb
pt_scene = vb.mapViewToScene(QtCore.QPointF(0.35, 10))
w.on_hover_runs(pt_scene); wait(0.2)
w.grab().save("shot_runs.png")
w.tabs.setCurrentIndex(0); w.sp_spin.setValue(0); wait(2.5)
w.backend.bump(); wait(1.5)
w.pause_btn.setChecked(True); wait(0.1)
vb = w.ls.angle.vb
r = vb.viewRange(); x = r[0][0] + 0.75*(r[0][1]-r[0][0])
w.on_hover_live(vb.mapViewToScene(QtCore.QPointF(x, 0))); wait(0.2)
w.grab().save("shot_live.png")
print("rate:", w.rate_label.text(), "live n:", w.live.n)
# fps check
w.pause_btn.setChecked(False)
t0=time.perf_counter(); n0=0
cnt=[0]
orig=w.redraw_live
def counted():
    cnt[0]+=1; orig()
w.redraw_live=counted
wait(3); print("redraws/s", cnt[0]/3)
w.terms_cb.setChecked(False); wait(0.5); w.grab().save("shot_noterms.png"); w.terms_cb.setChecked(True); wait(0.5); w.grab().save("shot_terms_back.png")
w.tabs.setCurrentIndex(2); wait(0.1); w.grab().save("shot_console.png")
w.close()
# kill / reset in demo
app2 = app
w2 = pt.MainWindow(); w2.show(); wait(1.0)
w2.kill(); wait(1.0); print("after kill motor:", w2.read_labels["motor"].text(), "angle:", w2.read_labels["angle"].text())
w2.backend.reset(); wait(1.5); print("after reset angle:", w2.read_labels["angle"].text())
w2.dots_cb.setChecked(True); wait(0.3); print("dots ok"); w2.close()
