#!/bin/sh
# Phone app 3.0 test on the PC (JVM). Usage: ANDROID_JAR=path/to/android.jar sh run_test.sh [fw_out folder]
# fw_out = real firmware output from Drone v3.0's host_tests/run_tests.sh (run that first).
# android.jar (Android SDK, platforms/android-33) is only needed to compile; the tested methods are plain Java.
cd "$(dirname "$0")"
ANDROID_JAR=${ANDROID_JAR:-android.jar}
rm -rf cls && mkdir cls
APP=../android_app/app/src/main/java/edu/dtcc/elc275/tuner/MainActivity.java      # repo layout
[ -f "$APP" ] || APP=../app/src/main/java/edu/dtcc/elc275/tuner/MainActivity.java   # working copy
javac -Xlint:-options -source 8 -target 8 -cp "$ANDROID_JAR" -d cls "$APP" AppTest.java
FW_OUT=${1:-../../../../firmware/03_drone/v3.0_two_pid_roll_pitch/host_tests/fw_out}   # repo layout
java -cp "cls:$ANDROID_JAR" edu.dtcc.elc275.tuner.AppTest "$FW_OUT"
rm -rf cls
