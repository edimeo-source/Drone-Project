# ==============================================================================================
#  ELC275 DRONE PROJECT  -  Ethan DiMeo
#  File    : tools/pc_pid_tuner/v1_serial_2026-11-11/tests/perftest.py
#  What    : PC-side test for PC PID tuner v1 (serial) (no board needed)
#  Date    : 2026-11-11
#  FUNCTIONS: wait(), counted()
# ==============================================================================================

import sys, time, cProfile, pstats
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gui"))
import pid_tuner as pt
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg
aa = sys.argv[1] == "1"
pg.setConfigOptions(antialias=aa, background=pt.SURFACE, foreground=pt.TEXT2)
app = QtWidgets.QApplication(sys.argv)
pt.apply_theme(app)
w = pt.MainWindow(); w.resize(1440, 900); w.show()
def wait(sec):
    end = time.perf_counter()+sec
    while time.perf_counter() < end:
        app.processEvents(); time.sleep(0.002)
wait(11)
cnt=[0]; orig=w.tick
ticks=[]
def counted():
    t=time.perf_counter(); orig(); ticks.append(time.perf_counter()-t)
w.timer.timeout.disconnect(); w.timer.timeout.connect(counted)
pr=cProfile.Profile(); pr.enable(); wait(3); pr.disable()
print("aa",aa,"ticks/s", len(ticks)/3, "mean tick ms", 1000*sum(ticks)/len(ticks))
pstats.Stats(pr).sort_stats("cumulative").print_stats(12)
