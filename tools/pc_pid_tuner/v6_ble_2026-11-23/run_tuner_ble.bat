@echo off
rem ELC275 PID Tuner - double-click to run
cd /d "%~dp0"

set PY=py
where py >nul 2>nul || set PY=python
%PY% --version >nul 2>nul
if errorlevel 1 (
    echo Python is not installed.
    echo Get it from https://www.python.org/downloads/ and tick "Add python.exe to PATH".
    pause
    exit /b 1
)

%PY% -c "import PySide6, pyqtgraph, numpy, serial, bleak" >nul 2>nul
if errorlevel 1 (
    echo First run: installing PySide6, pyqtgraph, numpy, pyserial, bleak - takes a minute...
    %PY% -m pip install --user PySide6 pyqtgraph numpy pyserial bleak
)

%PY% pid_tuner_ble.py
if errorlevel 1 pause
