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

%PY% -c "import PySide6, pyqtgraph, numpy, serial" >nul 2>nul
if errorlevel 1 (
    echo First run: installing PySide6, pyqtgraph, numpy, pyserial - takes a minute...
    %PY% -m pip install --user PySide6 pyqtgraph numpy pyserial
)

%PY% pid_tuner.py
if errorlevel 1 pause
