#!/bin/sh
# Rebuild DroneTuner.apk: javac -> dx (dex) -> aapt2 (manifest + icon) -> pack + align -> sign v1/v2/v3 -> verify
set -e
cd /home/claude/apkbuild
APP=/home/claude/v2/android_app/app/src/main
AAPT2=pkgs/aapt/package/bin/x64/linux/aapt2
rm -rf out /tmp/jc && mkdir -p out/res /tmp/jc
javac -Xlint:-options -source 8 -target 8 -bootclasspath pkgs/android-33.jar -d /tmp/jc $APP/java/edu/dtcc/elc275/tuner/MainActivity.java 2>&1 | grep -v "JAVA_TOOL\|^Note" || true
java -jar dxbuild/dx.jar --dex --min-sdk-version=24 --output=out/classes.dex /tmp/jc 2>&1 | grep -v JAVA_TOOL || true
$AAPT2 compile --dir $APP/res -o out/res/
$AAPT2 link -o out/base.apk -I pkgs/android-33.jar --manifest $APP/AndroidManifest.xml out/res/*.flat --min-sdk-version 24 --target-sdk-version 33 --version-code 2 --version-name 2.0
python3 -I scripts/pack.py out/base.apk out/classes.dex out/unsigned.apk > /dev/null
node scripts/sign.mjs out/unsigned.apk out/DroneTuner.apk keys/key.pem keys/cert.pem
python3 -I -c "import sys; sys.path.insert(0,'verify'); sys.argv=['apksigtool','verify','out/DroneTuner.apk']; import apksigtool; sys.exit(apksigtool.main())" 2>&1 | grep -v PROTOTYPE
jarsigner -verify out/DroneTuner.apk 2>&1 | grep -v JAVA_TOOL | head -1
