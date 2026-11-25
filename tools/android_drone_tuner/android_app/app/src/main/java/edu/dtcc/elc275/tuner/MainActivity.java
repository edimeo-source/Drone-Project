// =============================================================================================
//  ELC275 DRONE PROJECT  -  Ethan DiMeo
//  File    : tools/android_drone_tuner/android_app/app/src/main/java/edu/dtcc/elc275/tuner/MainActivity.java
//  What    : Android Drone Tuner app (the whole app is this one file), for Drone.X v2.0
//  Date    : 2026-11-23
//  Status  : APK built and signature-checked; not yet tested on a real phone with the drone.
//  METHODS: onCreate(), onDestroy(), onClick(), handleMessage(), armClicked(), kill(),
//    sendGains(), sendBase(), runClicked(), connectClicked(), onRequestPermissionsResult(),
//    onScanResult(), onScanFailed(), found(), stopScan(), onConnectionStateChange(),
//    onServicesDiscovered(), onDescriptorWrite(), onCharacteristicWrite(),
//    onCharacteristicChanged(), disconnect(), send(), pump(), run(), received(), handleLine(),
//    showProgress(), finishRun(), metrics(), shareCsv(), updateButtons(), save(), addLog(),
//    note(), clamp(), num(), dp(), text(), button(), number(), labeled(), weight(), row(),
//    card(), Plot(), addLive(), setRun(), onDraw(), panel(), path(), fmt()
//  Every method below has a '// ---- name()' title line with what it does.

// =============================================================================
//  ELC275 Drone Tuner (Android)          Ethan DiMeo          firmware 2.0
//
//  What it does:
//    - connects to the Adafruit Bluefruit LE SPI Friend on the drone (BLE, Nordic UART)
//    - ARM / KILL, with a heartbeat "H" every 100 ms while armed
//      (the drone cuts motor power if it hears no H for 500 ms, so closing the app,
//       walking out of range or the phone dying all stop the motors)
//    - sends the PID gains (K), base pulses (B), zero (Z), calibrate (C), hold (G)
//    - starts a step test (R). The drone records every 4 ms sample into arrays, sends
//      NOTHING while the motors run (professor's rule), stops the motors at the end,
//      then sends the arrays here. This app draws them and works out the step numbers.
//
//  Nothing is invented here: every number on screen came from the drone.
//  The only math done on the phone:
//    front pulse = base_front + u,  back pulse = base_back - u   (same mixer as the PIC,
//    floor 1100 us, ceiling 2000 us), and the step-response numbers at the bottom.
//
//  Bluetooth facts (why the data takes a while):
//    BLE packets carry 20 bytes of text. The Bluefruit gets about 100-300 bytes/s through
//    to the phone, so one sample (~8 bytes) every 4 ms (= 2000 bytes/s) can't be sent live.
//    That is why the drone stores the run first and sends it after.
// =============================================================================
package edu.dtcc.elc275.tuner;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanRecord;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;
import android.os.SystemClock;
import android.text.InputType;
import android.view.Gravity;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.Locale;
import java.util.UUID;

public class MainActivity extends Activity implements View.OnClickListener, Handler.Callback {

    // ---- Nordic UART service (the Bluefruit's "BLE UART") ----
    static final UUID NUS    = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    static final UUID NUS_TX = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // phone writes commands here
    static final UUID NUS_RX = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // drone's text arrives here
    static final UUID CCCD   = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"); // turns notifications on

    static final int HEARTBEAT_MS = 100;    // drone times out after 500 ms with no H
    static final int PHONE_MAX_MS = 5000;   // longest step test the drone accepts from the phone
    static final float PULSE_IDLE = 1100f, PULSE_MAX = 2000f;   // same limits as control.c

    // ---- colors (same as the PC tuner) ----
    static final int BG = 0xff232322, CARD = 0xff1a1a19, INPUT = 0xff2d2d2b, BORDER = 0xff3a3a37;
    static final int TEXT = 0xfff0efec, TEXT2 = 0xffc3c2b7, MUTED = 0xff8a8980;
    static final int BLUE = 0xff3987e5, ORANGE = 0xffd95926, GREEN = 0xff199e70, YELLOW = 0xfffab219;
    static final int RED = 0xffd03b3b;

    // Bluetooth events arrive on a background thread. Each one is passed to the screen's
    // thread as a message (handleMessage below), so only one thread ever touches the state.
    final Handler ui = new Handler(Looper.getMainLooper(), this);
    static final int MSG_FOUND = 1, MSG_SCAN_FAILED = 2, MSG_SCAN_TIMEOUT = 3, MSG_LOST = 4,
                     MSG_READY = 5, MSG_WRITTEN = 6, MSG_DATA = 7, MSG_PUMP = 8;

    // ---- Bluetooth state ----
    BluetoothLeScanner scanner;
    BluetoothGatt gatt;
    BluetoothGattCharacteristic txChar;
    boolean scanning, connected, armed, writing;
    long writeStarted;
    final ArrayDeque<byte[]> writes = new ArrayDeque<>();      // BLE allows one write at a time: queue the rest
    final StringBuilder rx = new StringBuilder();

    // ---- step-test data from the drone ----
    float[] runAngle, runU;
    int runTotal, runPre, runGot;
    float runFrom, runTo, runBf, runBb;
    String runGains = "";
    boolean receiving;
    long recvStart;
    int recvBytes;

    // ---- views ----
    TextView linkText, stateText, pitchText, rollText, gainsInUse, baseInUse, progressText, metricsText, logText;
    Button connectBtn, armBtn, killBtn, runBtn, shareBtn;
    EditText kpIn, kiIn, kdIn, fromIn, toIn, msIn, bfIn, bbIn;
    Plot plot;
    final ArrayDeque<String> log = new ArrayDeque<>();

    // =========================================================================
    //  Screen
    // =========================================================================
    // ---- onCreate() -------------------------------------------------------------------------
    //      Builds the screen (buttons, gain boxes, graph, log) and restores the saved settings.
    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);   // kill switch stays visible
        getWindow().setStatusBarColor(BG);
        SharedPreferences p = getPreferences(MODE_PRIVATE);

        LinearLayout col = new LinearLayout(this);
        col.setOrientation(LinearLayout.VERTICAL);
        col.setPadding(dp(14), dp(12), dp(14), dp(24));
        col.setBackgroundColor(BG);

        // title + connection
        TextView title = text("ELC275 Drone Tuner", 22, TEXT);
        title.setTypeface(Typeface.DEFAULT_BOLD);
        connectBtn = button("Connect", INPUT, "connect");
        col.addView(row(weight(title, 1), connectBtn));
        linkText = text("Not connected", 14, MUTED);
        col.addView(linkText);

        // ARM + KILL
        armBtn = button("ARM", GREEN, "arm");
        killBtn = button("KILL", RED, "kill");
        killBtn.setTextSize(26);
        armBtn.setTextSize(20);
        LinearLayout big = row(weight(armBtn, 1), weight(killBtn, 2));
        big.getLayoutParams().height = dp(84);
        killBtn.getLayoutParams().height = dp(84);
        armBtn.getLayoutParams().height = dp(84);
        col.addView(big);

        // live state
        stateText = text("DISARMED", 16, TEXT2);
        stateText.setTypeface(Typeface.DEFAULT_BOLD);
        pitchText = text("pitch  --", 26, TEXT);
        rollText = text("roll  --", 16, TEXT2);
        col.addView(stateText);
        col.addView(row(weight(pitchText, 1), rollText));

        // graph
        plot = new Plot(this);
        col.addView(plot, new LinearLayout.LayoutParams(-1, dp(300)));

        // step test
        LinearLayout step = card(col, "Step test  (pitch only - the drone holds 'from' 1 s, then steps to 'to')");
        fromIn = number("from deg", p.getString("from", "0"));
        toIn = number("to deg", p.getString("to", "10"));
        msIn = number("record ms", p.getString("ms", "3000"));
        step.addView(row(labeled("from (deg)", fromIn), labeled("to (deg)", toIn), labeled("record (ms)", msIn)));
        runBtn = button("Run step test", BLUE, "run");
        shareBtn = button("Share CSV", INPUT, "share");
        shareBtn.setEnabled(false);
        step.addView(row(weight(runBtn, 2), weight(shareBtn, 1)));
        progressText = text("", 14, YELLOW);
        metricsText = text("", 14, TEXT);
        metricsText.setTypeface(Typeface.MONOSPACE);
        step.addView(progressText);
        step.addView(metricsText);

        // gains
        LinearLayout gains = card(col, "PID gains  (u = Kp e + Ki ∫e dt + Kd de/dt,  u in us)");
        kpIn = number("Kp", p.getString("kp", "1.0"));
        kiIn = number("Ki", p.getString("ki", "0.0"));
        kdIn = number("Kd", p.getString("kd", "0.0"));
        gains.addView(row(labeled("Kp", kpIn), labeled("Ki", kiIn), labeled("Kd", kdIn)));
        gains.addView(button("Send gains", INPUT, "gains"));
        gainsInUse = text("in use on the drone: --", 14, TEXT2);
        gains.addView(gainsInUse);

        // motors
        LinearLayout mot = card(col, "Motors  (front pair = ESC 1+2, back pair = ESC 3+4)");
        bfIn = number("front us", p.getString("bf", "1500"));
        bbIn = number("back us", p.getString("bb", "1500"));
        mot.addView(row(labeled("base front (us)", bfIn), labeled("base back (us)", bbIn)));
        mot.addView(button("Set base pulses", INPUT, "base"));
        baseInUse = text("in use on the drone: --", 14, TEXT2);
        mot.addView(baseInUse);
        mot.addView(row(
                weight(button("Zero (Z)", INPUT, ">Z"), 1),
                weight(button("Calibrate (C)", INPUT, ">C"), 1)));
        mot.addView(row(
                weight(button("PID hold (G)", INPUT, ">G"), 1),
                weight(button("Status", INPUT, ">STATUS"), 1)));

        // messages from the drone
        LinearLayout msgs = card(col, "Messages from the drone");
        logText = text("", 13, TEXT2);
        logText.setTypeface(Typeface.MONOSPACE);
        msgs.addView(logText);

        ScrollView sv = new ScrollView(this);
        sv.setBackgroundColor(BG);
        sv.addView(col);
        setContentView(sv);
        updateButtons();
        ui.postDelayed(heartbeat, HEARTBEAT_MS);
    }

    // ---- onDestroy() ------------------------------------------------------------------------
    //      App closing: KILL, disconnect and save.
    @Override
    protected void onDestroy() {
        if (armed) kill();                  // and the drone times out anyway once H stops
        ui.removeCallbacksAndMessages(null);
        stopScan();
        if (gatt != null) { gatt.disconnect(); gatt.close(); }
        super.onDestroy();
    }

    // =========================================================================
    //  Buttons (each button's tag says what it does; ">X" = just send X)
    // =========================================================================
    // ---- onClick() --------------------------------------------------------------------------
    //      Routes every button press to its action.
    @Override
    public void onClick(View v) {
        String a = (String) v.getTag();
        if (a.startsWith(">")) send(a.substring(1));
        else if (a.equals("connect")) connectClicked();
        else if (a.equals("arm")) armClicked();
        else if (a.equals("kill")) kill();
        else if (a.equals("run")) runClicked();
        else if (a.equals("share")) shareCsv();
        else if (a.equals("gains")) sendGains();
        else if (a.equals("base")) sendBase();
    }

    // Messages from the Bluetooth thread, handled here on the screen's thread
    // ---- handleMessage() --------------------------------------------------------------------
    //      Runs Bluetooth events on the UI thread (scan results, connect, lines received,
    //      errors).
    @Override
    public boolean handleMessage(android.os.Message m) {
        switch (m.what) {
            case MSG_FOUND:
                Object[] o = (Object[]) m.obj;
                found((BluetoothDevice) o[0], (String) o[1]);
                break;
            case MSG_SCAN_FAILED:
                scanning = false;
                linkText.setText("Scan failed (" + m.arg1 + ")");
                updateButtons();
                break;
            case MSG_SCAN_TIMEOUT:
                if (scanning) {
                    stopScan();
                    linkText.setText("No Bluefruit found. Is the drone powered?");
                    updateButtons();
                }
                break;
            case MSG_LOST:
                disconnect((String) m.obj);
                break;
            case MSG_READY:
                connected = true;
                linkText.setText("Connected to the drone");
                linkText.setTextColor(GREEN);
                updateButtons();
                send("STATUS");
                send("B");                  // B alone = report the base pulses
                break;
            case MSG_WRITTEN:
                writing = false;
                pump();
                break;
            case MSG_DATA:
                received((byte[]) m.obj);
                break;
            case MSG_PUMP:
                pump();
                break;
        }
        return true;
    }

    // ---- armClicked() -----------------------------------------------------------------------
    //      ARM button: sends ARM and starts the heartbeat (the drone answers ARMED or ARM
    //      refused).
    void armClicked() {
        if (!connected) { note("Connect first"); return; }
        send("ARM");
        armed = true;               // start the heartbeat now; the drone answers ARMED or DISARMED,ARM refused
        updateButtons();
    }

    // ---- kill() -----------------------------------------------------------------------------
    //      KILL: drops queued writes, sends X twice and stops the heartbeat (the drone also
    //      cuts power after 500 ms without H).
    void kill() {
        writes.clear();             // drop anything waiting: X goes out next
        armed = false;              // stop the heartbeat too (the drone also cuts power after 500 ms without H)
        send("X");
        send("X");
        stateText.setText("KILL sent");
        stateText.setTextColor(RED);
        updateButtons();
    }

    // ---- sendGains() ------------------------------------------------------------------------
    //      Sends the gains in the boxes: K,kp,ki,kd.
    void sendGains() {
        float kp = num(kpIn), ki = num(kiIn), kd = num(kdIn);
        if (Float.isNaN(kp) || Float.isNaN(ki) || Float.isNaN(kd)) { note("Gains must be numbers"); return; }
        send(String.format(Locale.US, "K,%.4f,%.4f,%.4f", kp, ki, kd));
        save();
    }

    // ---- sendBase() -------------------------------------------------------------------------
    //      Sends the base pulses: B,front,back.
    void sendBase() {
        float f = num(bfIn), b = num(bbIn);
        if (Float.isNaN(f) || Float.isNaN(b)) { note("Pulses must be numbers"); return; }
        send(String.format(Locale.US, "B,%d,%d", Math.round(f), Math.round(b)));
        save();
    }

    // ---- runClicked() -----------------------------------------------------------------------
    //      RUN button: sends R,from,to,ms; the drone records the run and sends it afterwards.
    void runClicked() {
        float from = num(fromIn), to = num(toIn), ms = num(msIn);
        if (Float.isNaN(from) || Float.isNaN(to) || Float.isNaN(ms)) { note("Step test needs numbers"); return; }
        if (ms > PHONE_MAX_MS) { ms = PHONE_MAX_MS; msIn.setText(String.valueOf(PHONE_MAX_MS)); }
        if (receiving) { note("Still receiving the last run"); return; }
        sendGains();                // run with the gains on screen
        send(String.format(Locale.US, "R,%.1f,%.1f,%d", from, to, Math.round(ms)));
        progressText.setText("Motors running - the drone sends nothing until the run ends");
        metricsText.setText("");
        save();
    }

    // =========================================================================
    //  Bluetooth: find, connect, send, receive
    // =========================================================================
    // ---- connectClicked() -------------------------------------------------------------------
    //      Connect button: asks for Bluetooth permission if needed, then scans for the
    //      Bluefruit.
    void connectClicked() {
        if (connected || gatt != null || scanning) { disconnect("Disconnected"); return; }
        String[] perms = Build.VERSION.SDK_INT >= 31
                ? new String[]{Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT}
                : new String[]{Manifest.permission.ACCESS_FINE_LOCATION};
        for (String s : perms) {
            if (checkSelfPermission(s) != PackageManager.PERMISSION_GRANTED) {
                requestPermissions(perms, 1);   // tap Connect again after allowing
                return;
            }
        }
        BluetoothManager bm = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        BluetoothAdapter ad = bm == null ? null : bm.getAdapter();
        if (ad == null || !ad.isEnabled()) { linkText.setText("Turn Bluetooth on, then tap Connect"); return; }
        scanner = ad.getBluetoothLeScanner();
        if (scanner == null) { linkText.setText("Bluetooth is starting, try again"); return; }
        ScanSettings set = new ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build();
        scanner.startScan(null, set, scanCb);
        scanning = true;
        linkText.setText("Searching for the Bluefruit... (location must be on for Android 11 and older)");
        ui.sendEmptyMessageDelayed(MSG_SCAN_TIMEOUT, 10000);
        updateButtons();
    }

    // ---- onRequestPermissionsResult() -------------------------------------------------------
    //      Bluetooth permission answered: scan if it was granted.
    @Override   // the user answered the permission prompt: carry on connecting if allowed
    public void onRequestPermissionsResult(int code, String[] perms, int[] results) {
        for (int r : results) {
            if (r != PackageManager.PERMISSION_GRANTED) {
                linkText.setText("Bluetooth permission is needed to find the drone");
                return;
            }
        }
        if (results.length > 0) connectClicked();
    }

    final ScanCallback scanCb = new ScanCallback() {
        // ---- onScanResult() -----------------------------------------------------------------
        //      A BLE device was seen: pick it if it advertises the Nordic UART service or is
        //      named Bluefruit.
        @Override
        public void onScanResult(int type, ScanResult r) {
            ScanRecord rec = r.getScanRecord();
            String name = rec == null ? null : rec.getDeviceName();
            boolean uart = rec != null && rec.getServiceUuids() != null
                    && rec.getServiceUuids().contains(new ParcelUuid(NUS));
            boolean named = name != null && name.toLowerCase(Locale.US).contains("bluefruit");
            if (uart || named) {
                BluetoothDevice dev = r.getDevice();
                String shown = name == null ? dev.getAddress() : name;
                ui.obtainMessage(MSG_FOUND, new Object[]{dev, shown}).sendToTarget();
            }
        }
        // ---- onScanFailed() -----------------------------------------------------------------
        //      The BLE scan failed: report it.
        @Override
        public void onScanFailed(int code) { ui.obtainMessage(MSG_SCAN_FAILED, code, 0).sendToTarget(); }
    };

    // ---- found() ----------------------------------------------------------------------------
    //      Bluefruit found: stop scanning and connect to it.
    void found(BluetoothDevice dev, String name) {
        if (!scanning) return;
        stopScan();
        ui.removeMessages(MSG_SCAN_TIMEOUT);
        linkText.setText("Connecting to " + name + "...");
        gatt = dev.connectGatt(this, false, gattCb, BluetoothDevice.TRANSPORT_LE);
    }

    // ---- stopScan() -------------------------------------------------------------------------
    //      Stops the BLE scan.
    void stopScan() {
        if (scanning && scanner != null) {
            try { scanner.stopScan(scanCb); } catch (Exception ignored) { }
        }
        scanning = false;
    }

    final BluetoothGattCallback gattCb = new BluetoothGattCallback() {
        // ---- onConnectionStateChange() ------------------------------------------------------
        //      Connected: discover services. Disconnected: report it.
        @Override
        public void onConnectionStateChange(BluetoothGatt g, int status, int state) {
            if (state == BluetoothProfile.STATE_CONNECTED) g.discoverServices();
            else if (state == BluetoothProfile.STATE_DISCONNECTED) ui.obtainMessage(MSG_LOST, "Link lost - the drone turns its motors off by itself").sendToTarget();
        }
        // ---- onServicesDiscovered() ---------------------------------------------------------
        //      Finds the Nordic UART service and turns on notifications for its TX
        //      characteristic.
        @Override
        public void onServicesDiscovered(BluetoothGatt g, int status) {
            BluetoothGattService s = g.getService(NUS);
            if (s == null) { ui.obtainMessage(MSG_LOST, "That device has no BLE UART").sendToTarget(); return; }
            BluetoothGattCharacteristic in = s.getCharacteristic(NUS_RX);
            txChar = s.getCharacteristic(NUS_TX);
            g.setCharacteristicNotification(in, true);              // drone -> phone text
            BluetoothGattDescriptor d = in.getDescriptor(CCCD);
            d.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
            g.writeDescriptor(d);
        }
        // ---- onDescriptorWrite() ------------------------------------------------------------
        //      Notifications are on: the link is ready.
        @Override
        public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int status) {
            ui.sendEmptyMessage(MSG_READY);
        }
        // ---- onCharacteristicWrite() --------------------------------------------------------
        //      One 20-byte write finished: send the next piece.
        @Override
        public void onCharacteristicWrite(BluetoothGatt g, BluetoothGattCharacteristic c, int status) {
            ui.sendEmptyMessage(MSG_WRITTEN);
        }
        // ---- onCharacteristicChanged() ------------------------------------------------------
        //      Bytes arrived from the drone: pass them to received().
        @Override   // Android 12 and older
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c) {
            ui.obtainMessage(MSG_DATA, c.getValue().clone()).sendToTarget();
        }
        // ---- onCharacteristicChanged() ------------------------------------------------------
        //      Bytes arrived from the drone: pass them to received().
        @Override   // Android 13 and newer
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c, byte[] value) {
            ui.obtainMessage(MSG_DATA, value.clone()).sendToTarget();
        }
    };

    // ---- disconnect() -----------------------------------------------------------------------
    //      Closes the BLE link and shows why.
    void disconnect(String why) {
        stopScan();
        if (gatt != null) { gatt.disconnect(); gatt.close(); gatt = null; }
        connected = false;
        armed = false;
        writing = false;
        writes.clear();
        if (receiving) { receiving = false; progressText.setText("Link lost - the drone drops the run data"); }
        linkText.setText(why);
        linkText.setTextColor(MUTED);
        stateText.setText("DISARMED");
        stateText.setTextColor(TEXT2);
        updateButtons();
    }

    // one command = one line. BLE writes carry at most 20 bytes, so long lines go in pieces.
    // ---- send() -----------------------------------------------------------------------------
    //      Queues a command line for the drone (sent in 20-byte pieces).
    void send(String cmd) {
        if (!connected) return;
        byte[] all = (cmd + "\n").getBytes(StandardCharsets.US_ASCII);
        for (int i = 0; i < all.length; i += 20) {
            byte[] part = new byte[Math.min(20, all.length - i)];
            System.arraycopy(all, i, part, 0, part.length);
            writes.add(part);
        }
        if (!cmd.equals("H")) addLog("> " + cmd);
        pump();
    }

    // ---- pump() -----------------------------------------------------------------------------
    //      Sends the next queued 20-byte piece if no write is in progress.
    void pump() {
        if (!connected || writing || writes.isEmpty() || txChar == null) return;
        byte[] b = writes.peek();
        txChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
        txChar.setValue(b);
        if (gatt.writeCharacteristic(txChar)) {
            writes.poll();
            writing = true;
            writeStarted = SystemClock.uptimeMillis();
        } else {
            ui.sendEmptyMessageDelayed(MSG_PUMP, 20);   // Bluetooth stack busy: try again in 20 ms
        }
    }

    // H every 100 ms while armed. The drone counts H only from the link that armed it.
    final Runnable heartbeat = new Runnable() {
        // ---- run() --------------------------------------------------------------------------
        //      Heartbeat timer: sends H every 100 ms while armed.
        @Override
        public void run() {
            if (writing && SystemClock.uptimeMillis() - writeStarted > 1000) { writing = false; pump(); }  // lost callback
            if (connected && armed && writes.size() < 3) send("H");
            ui.postDelayed(this, HEARTBEAT_MS);
        }
    };

    // ---- received() -------------------------------------------------------------------------
    //      Joins received bytes into text lines.
    void received(byte[] v) {
        if (receiving) recvBytes += v.length;
        for (byte c : v) {
            if (c == '\n') {
                String line = rx.toString().trim();
                rx.setLength(0);
                if (line.length() > 0) handleLine(line);
            } else if (c != '\r' && rx.length() < 200) {
                rx.append((char) c);
            }
        }
    }

    // =========================================================================
    //  Lines from the drone (see tuner.c and main.c)
    // =========================================================================
    // ---- handleLine() -----------------------------------------------------------------------
    //      Runs one line from the drone: live data, run arrays, status (ARMED, DISARMED, ...).
    void handleLine(String s) {
        String[] f = s.split(",");
        try {
            switch (f[0]) {
                case "P":           // P,pitch x10,roll x10,armed   (only while the motors are off)
                    float pa = Integer.parseInt(f[1]) / 10f, ra = Integer.parseInt(f[2]) / 10f;
                    pitchText.setText(String.format(Locale.US, "pitch %+.1f°", pa));
                    rollText.setText(String.format(Locale.US, "roll %+.1f°", ra));
                    if (!receiving) plot.addLive(pa);
                    return;
                case "A":           // A,kp,ki,kd
                    gainsInUse.setText("in use on the drone:  Kp " + f[1] + "   Ki " + f[2] + "   Kd " + f[3]);
                    return;
                case "BASE":        // BASE,front,back
                    baseInUse.setText("in use on the drone:  front " + f[1] + " us   back " + f[2] + " us");
                    runBf = Float.parseFloat(f[1]);
                    runBb = Float.parseFloat(f[2]);
                    return;
                case "RUN_BEGIN":   // RUN_BEGIN,samples,pre,from,to,base_f,base_b,kp,ki,kd
                    if (f.length < 10) break;
                    runTotal = Integer.parseInt(f[1]);
                    runPre = Integer.parseInt(f[2]);
                    runFrom = Float.parseFloat(f[3]);
                    runTo = Float.parseFloat(f[4]);
                    runBf = Float.parseFloat(f[5]);
                    runBb = Float.parseFloat(f[6]);
                    runGains = "Kp " + f[7] + "  Ki " + f[8] + "  Kd " + f[9];
                    runAngle = new float[runTotal];
                    runU = new float[runTotal];
                    runGot = 0;
                    receiving = true;
                    recvStart = SystemClock.uptimeMillis();
                    recvBytes = s.length() + 1;
                    shareBtn.setEnabled(false);
                    showProgress();
                    return;
                case "RD":          // RD,k,a,u,a,u,...  a = angle x 10, u = PID output (us)
                    if (!receiving) return;
                    int k = Integer.parseInt(f[1]);
                    for (int i = 2; i + 1 < f.length && k < runTotal; i += 2, k++) {
                        runAngle[k] = Integer.parseInt(f[i]) / 10f;
                        runU[k] = Integer.parseInt(f[i + 1]);
                        runGot++;
                    }
                    showProgress();
                    return;
                case "RUN_END":
                    if (receiving) finishRun();
                    else progressText.setText("Run stopped - no data");
                    return;
            }
        } catch (RuntimeException e) {
            addLog("? " + s);       // garbled line: show it, don't guess
            return;
        }
        if (s.startsWith("ARMED")) {
            armed = true;
            stateText.setText("ARMED - wait 2 s for the ESCs, then start");
            stateText.setTextColor(YELLOW);
        } else if (s.startsWith("DISARMED")) {
            armed = false;          // the drone turned the motors off: stop the heartbeat
            stateText.setText(s.replace(",", "  "));
            stateText.setTextColor(TEXT2);
        } else if (s.startsWith("STATUS,armed")) {
            stateText.setText("ARMED");
            stateText.setTextColor(YELLOW);
        }
        addLog(s);
        updateButtons();
    }

    // ---- showProgress() ---------------------------------------------------------------------
    //      Shows how much of the recorded run has arrived.
    void showProgress() {
        float secs = (SystemClock.uptimeMillis() - recvStart) / 1000f;
        float rate = secs > 0.5f ? recvBytes / secs : 0;
        String left = rate > 0 && runGot > 0 ? String.format(Locale.US, ", about %.0f s left",
                secs * (runTotal - runGot) / runGot) : "";
        progressText.setText(String.format(Locale.US, "Receiving %d / %d samples  -  %.0f bytes/s measured%s",
                runGot, runTotal, rate, left));
    }

    // ---- finishRun() ------------------------------------------------------------------------
    //      Whole run received: draws it and works out rise time, overshoot, settling and error.
    void finishRun() {
        receiving = false;
        float secs = (SystemClock.uptimeMillis() - recvStart) / 1000f;
        progressText.setText(String.format(Locale.US, "Got %d of %d samples (%d bytes in %.1f s = %.0f bytes/s)",
                runGot, runTotal, recvBytes, secs, recvBytes / Math.max(secs, 0.001f)));
        float[] t = new float[runTotal], sp = new float[runTotal], fr = new float[runTotal], bk = new float[runTotal];
        for (int k = 0; k < runTotal; k++) {
            t[k] = (k - runPre) * 0.004f;                    // 4 ms per sample, t = 0 at the step
            sp[k] = k < runPre ? runFrom : runTo;
            fr[k] = clamp(runBf + runU[k], PULSE_IDLE, PULSE_MAX);   // same mixer as control.c
            bk[k] = clamp(runBb - runU[k], PULSE_IDLE, PULSE_MAX);
        }
        plot.setRun(t, runAngle, sp, fr, bk);
        metricsText.setText(runGains + "\n" + metrics(t, runAngle, fr, bk));
        shareBtn.setEnabled(runGot == runTotal);
    }

    // Same definitions as the PC tuner's step_metrics()
    // ---- metrics() --------------------------------------------------------------------------
    //      Works out the step-response numbers (rise, overshoot, settling, error, saturation)
    //      as text.
    String metrics(float[] t, float[] a, float[] fr, float[] bk) {
        int n0 = runPre, n = runTotal - n0;
        if (n < 10) return "";
        float[] y = new float[n];
        for (int i = 0; i < n; i++) {           // 7-sample (28 ms) average so noise can't fake the numbers
            float sum = 0; int c = 0;
            for (int j = i - 3; j <= i + 3; j++) if (j >= 0 && j < n) { sum += a[n0 + j]; c++; }
            y[i] = sum / c;
        }
        int sat = 0, tail = 0; float tailSum = 0;
        for (int i = 0; i < n; i++) {
            if (fr[n0 + i] >= PULSE_MAX - 0.5f || bk[n0 + i] >= PULSE_MAX - 0.5f
                    || fr[n0 + i] <= PULSE_IDLE + 0.5f || bk[n0 + i] <= PULSE_IDLE + 0.5f) sat++;
            if (t[n0 + i] >= t[runTotal - 1] - 0.5f) { tailSum += y[i]; tail++; }
        }
        float fin = tailSum / tail;
        StringBuilder m = new StringBuilder();
        m.append(String.format(Locale.US, "steady-state error %+.2f deg   at limits %d%%\n", runTo - fin, 100 * sat / n));
        float step = runTo - runFrom;
        if (Math.abs(step) < 0.5f) return m.append("(no step: holding only)").toString();
        float t10 = -1, t90 = -1, peak = -1e9f;
        for (int i = 0; i < n; i++) {
            float x = (y[i] - runFrom) / step;          // 0 = start, 1 = setpoint
            if (t10 < 0 && x >= 0.1f) t10 = t[n0 + i];
            if (t90 < 0 && x >= 0.9f) t90 = t[n0 + i];
            peak = Math.max(peak, x);
        }
        if (t90 >= 0) m.append(String.format(Locale.US, "rise 10-90%% %.3f s   overshoot %.0f%%\n", t90 - t10, Math.max(0, (peak - 1) * 100)));
        else m.append(String.format(Locale.US, "never reached 90%% (got to %.0f%%)\n", peak * 100));
        float band = Math.max(0.02f * Math.abs(fin - runFrom), 0.5f);
        int last = -1;
        for (int i = 0; i < n; i++) if (Math.abs(y[i] - fin) > band) last = i;
        if (last == n - 1) m.append("settling: not settled");
        else m.append(String.format(Locale.US, "settling (within %.1f deg) %.3f s", band, last < 0 ? 0f : t[n0 + last + 1]));
        return m.toString();
    }

    // ---- shareCsv() -------------------------------------------------------------------------
    //      Shares the last run as a CSV file (email, Drive, ...).
    void shareCsv() {
        StringBuilder c = new StringBuilder();
        c.append("# ELC275 drone step test, ").append(runGains)
         .append(String.format(Locale.US, ", base front %.0f us, base back %.0f us, from %.1f to %.1f deg\n", runBf, runBb, runFrom, runTo));
        c.append("t_s,pitch_deg,setpoint_deg,u_us,front_us,back_us\n");
        for (int k = 0; k < runTotal; k++) {
            c.append(String.format(Locale.US, "%.3f,%.1f,%.1f,%.0f,%.0f,%.0f\n", (k - runPre) * 0.004f, runAngle[k],
                    k < runPre ? runFrom : runTo, runU[k],
                    clamp(runBf + runU[k], PULSE_IDLE, PULSE_MAX), clamp(runBb - runU[k], PULSE_IDLE, PULSE_MAX)));
        }
        Intent i = new Intent(Intent.ACTION_SEND);
        i.setType("text/plain");
        i.putExtra(Intent.EXTRA_SUBJECT, "Drone step test " + runGains);
        i.putExtra(Intent.EXTRA_TEXT, c.toString());
        startActivity(Intent.createChooser(i, "Send the CSV to..."));
    }

    // =========================================================================
    //  Small helpers
    // =========================================================================
    // ---- updateButtons() --------------------------------------------------------------------
    //      Enables/disables buttons to match the connection and arming state.
    void updateButtons() {
        connectBtn.setText(connected || gatt != null || scanning ? "Disconnect" : "Connect");
        armBtn.setEnabled(connected && !armed);
        armBtn.setAlpha(armBtn.isEnabled() ? 1f : 0.4f);
        runBtn.setEnabled(connected && armed);
        runBtn.setAlpha(runBtn.isEnabled() ? 1f : 0.4f);
        if (!connected) { pitchText.setText("pitch  --"); rollText.setText("roll  --"); }
    }

    // ---- save() -----------------------------------------------------------------------------
    //      Saves the gains and settings.
    void save() {
        getPreferences(MODE_PRIVATE).edit()
                .putString("kp", kpIn.getText().toString()).putString("ki", kiIn.getText().toString())
                .putString("kd", kdIn.getText().toString()).putString("from", fromIn.getText().toString())
                .putString("to", toIn.getText().toString()).putString("ms", msIn.getText().toString())
                .putString("bf", bfIn.getText().toString()).putString("bb", bbIn.getText().toString()).apply();
    }

    // ---- addLog() ---------------------------------------------------------------------------
    //      Adds a line to the on-screen log.
    void addLog(String s) {
        log.addLast(s);
        while (log.size() > 12) log.removeFirst();
        StringBuilder b = new StringBuilder();
        for (String x : log) b.append(x).append('\n');
        logText.setText(b.toString().trim());
    }

    // ---- note() -----------------------------------------------------------------------------
    //      Adds a highlighted note to the log.
    void note(String s) { addLog("! " + s); }

    // ---- clamp() ----------------------------------------------------------------------------
    //      Limits x to lo..hi.
    static float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

    // ---- num() ------------------------------------------------------------------------------
    //      Reads a number from a text box (0 if empty or invalid).
    static float num(EditText e) {
        try { return Float.parseFloat(e.getText().toString().trim()); } catch (NumberFormatException x) { return Float.NaN; }
    }

    // ---- dp() -------------------------------------------------------------------------------
    //      Converts dp units to screen pixels.
    int dp(int v) { return Math.round(v * getResources().getDisplayMetrics().density); }

    // ---- text() -----------------------------------------------------------------------------
    //      Makes a text label (size in sp, colour).
    TextView text(String s, int sp, int color) {
        TextView t = new TextView(this);
        t.setText(s);
        t.setTextSize(sp);
        t.setTextColor(color);
        t.setPadding(0, dp(4), 0, dp(4));
        return t;
    }

    // ---- button() ---------------------------------------------------------------------------
    //      Makes a styled button that runs 'action' when pressed.
    Button button(String s, int bg, String action) {
        Button b = new Button(this);
        b.setText(s);
        b.setAllCaps(false);
        b.setTextColor(Color.WHITE);
        b.setTextSize(16);
        GradientDrawable d = new GradientDrawable();
        d.setColor(bg);
        d.setCornerRadius(dp(8));
        if (bg == INPUT) d.setStroke(dp(1), BORDER);
        b.setBackground(d);
        b.setMinHeight(dp(48));
        b.setTag(action);
        b.setOnClickListener(this);
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(-2, -2);
        lp.setMargins(dp(4), dp(4), dp(4), dp(4));
        b.setLayoutParams(lp);
        return b;
    }

    // ---- number() ---------------------------------------------------------------------------
    //      Makes a number entry box with a hint and a starting value.
    EditText number(String hint, String value) {
        EditText e = new EditText(this);
        e.setHint(hint);
        e.setText(value);
        e.setTextColor(TEXT);
        e.setHintTextColor(MUTED);
        e.setTextSize(18);
        e.setSingleLine(true);
        e.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL | InputType.TYPE_NUMBER_FLAG_SIGNED);
        GradientDrawable d = new GradientDrawable();
        d.setColor(INPUT);
        d.setCornerRadius(dp(6));
        d.setStroke(dp(1), BORDER);
        e.setBackground(d);
        e.setPadding(dp(10), dp(8), dp(10), dp(8));
        return e;
    }

    // ---- labeled() --------------------------------------------------------------------------
    //      Puts a small label above an entry box.
    View labeled(String label, EditText e) {
        LinearLayout c = new LinearLayout(this);
        c.setOrientation(LinearLayout.VERTICAL);
        c.addView(text(label, 13, MUTED));
        c.addView(e);
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(0, -2, 1);
        lp.setMargins(dp(4), 0, dp(4), 0);
        c.setLayoutParams(lp);
        return c;
    }

    // ---- weight() ---------------------------------------------------------------------------
    //      Gives a view a share (w) of its row's width.
    View weight(View v, float w) {
        LinearLayout.LayoutParams old = (LinearLayout.LayoutParams) v.getLayoutParams();
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(0, -2, w);
        if (old != null) lp.setMargins(old.leftMargin, old.topMargin, old.rightMargin, old.bottomMargin);
        v.setLayoutParams(lp);
        return v;
    }

    // ---- row() ------------------------------------------------------------------------------
    //      Puts views side by side in one row.
    LinearLayout row(View... vs) {
        LinearLayout r = new LinearLayout(this);
        r.setOrientation(LinearLayout.HORIZONTAL);
        r.setGravity(Gravity.CENTER_VERTICAL);
        r.setLayoutParams(new LinearLayout.LayoutParams(-1, -2));
        for (View v : vs) r.addView(v);
        return r;
    }

    // ---- card() -----------------------------------------------------------------------------
    //      Makes a titled card section on the screen.
    LinearLayout card(LinearLayout parent, String title) {
        LinearLayout c = new LinearLayout(this);
        c.setOrientation(LinearLayout.VERTICAL);
        c.setPadding(dp(10), dp(8), dp(10), dp(10));
        GradientDrawable d = new GradientDrawable();
        d.setColor(CARD);
        d.setCornerRadius(dp(10));
        d.setStroke(dp(1), BORDER);
        c.setBackground(d);
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(-1, -2);
        lp.setMargins(0, dp(12), 0, 0);
        parent.addView(c, lp);
        TextView t = text(title, 14, TEXT2);
        t.setTypeface(Typeface.DEFAULT_BOLD);
        c.addView(t);
        return c;
    }

    // =========================================================================
    //  Graph: top = pitch and setpoint (deg), bottom = front and back pulses (us)
    // =========================================================================
    // ---- Plot() -----------------------------------------------------------------------------
    //      The graph view: live pitch, or a recorded run (angle/setpoint and front/back
    //      pulses).
    static class Plot extends View {
        final Paint line = new Paint(Paint.ANTI_ALIAS_FLAG), dash = new Paint(Paint.ANTI_ALIAS_FLAG);
        final Paint grid = new Paint(), label = new Paint(Paint.ANTI_ALIAS_FLAG);
        float[] t, a, sp, fr, bk;               // a recorded run
        final float[] live = new float[240];    // live pitch while the motors are off
        int liveN;
        final float den;

        Plot(Context c) {
            super(c);
            den = c.getResources().getDisplayMetrics().density;
            line.setStyle(Paint.Style.STROKE);
            line.setStrokeWidth(2 * den);
            dash.setStyle(Paint.Style.STROKE);
            dash.setStrokeWidth(1.5f * den);
            dash.setColor(TEXT2);
            dash.setPathEffect(new DashPathEffect(new float[]{8 * den, 6 * den}, 0));
            grid.setColor(BORDER);
            grid.setStrokeWidth(den);
            label.setColor(MUTED);
            label.setTextSize(11 * den);
        }

        // ---- addLive() ----------------------------------------------------------------------
        //      Adds one live pitch sample to the graph.
        void addLive(float pitch) {
            if (liveN == live.length) { System.arraycopy(live, 1, live, 0, live.length - 1); liveN--; }
            live[liveN++] = pitch;
            if (t == null) invalidate();
        }

        // ---- setRun() -----------------------------------------------------------------------
        //      Shows a recorded run on the graph.
        void setRun(float[] t, float[] a, float[] sp, float[] fr, float[] bk) {
            this.t = t; this.a = a; this.sp = sp; this.fr = fr; this.bk = bk;
            invalidate();
        }

        // ---- onDraw() -----------------------------------------------------------------------
        //      Draws the graph.
        @Override
        protected void onDraw(Canvas c) {
            float w = getWidth(), h = getHeight(), pad = 6 * den, left = 44 * den;
            GradientDrawable bg = new GradientDrawable();
            bg.setColor(CARD);
            bg.setCornerRadius(10 * den);
            bg.setStroke((int) den, BORDER);
            bg.setBounds(0, 0, (int) w, (int) h);
            bg.draw(c);
            if (t == null) {                            // live view
                float[] y = new float[liveN];
                float[] x = new float[liveN];
                for (int i = 0; i < liveN; i++) { x[i] = i; y[i] = live[i]; }
                panel(c, "live pitch (deg), 4 per second, motors off", x, new float[][]{y}, new int[]{BLUE}, null,
                        left, pad, w - pad, h - pad, false);
                return;
            }
            float split = h * 0.58f;
            panel(c, "pitch (deg)  - - setpoint", t, new float[][]{a}, new int[]{BLUE}, sp, left, pad, w - pad, split - pad, true);
            panel(c, "front us (orange)  back us (green)", t, new float[][]{fr, bk}, new int[]{ORANGE, GREEN}, null,
                    left, split + pad, w - pad, h - pad, true);
        }

        // ---- panel() ------------------------------------------------------------------------
        //      Draws one graph panel with its title, curves and axis numbers.
        void panel(Canvas c, String title, float[] x, float[][] ys, int[] colors, float[] dashed,
                   float x0, float y0, float x1, float y1, boolean timeAxis) {
            c.drawText(title, x0, y0 + 12 * den, label);
            float top = y0 + 18 * den, bottom = y1 - (timeAxis ? 14 * den : 2 * den);
            if (x.length < 2) { c.drawText("waiting for data", x0, (top + bottom) / 2, label); return; }
            float lo = Float.MAX_VALUE, hi = -Float.MAX_VALUE;
            for (float[] y : ys) for (float v : y) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
            if (dashed != null) for (float v : dashed) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
            if (hi - lo < 1) { hi += 0.5f; lo -= 0.5f; }
            float m = (hi - lo) * 0.08f;
            lo -= m; hi += m;
            float xa = x[0], xb = x[x.length - 1];
            c.drawLine(x0, top, x0, bottom, grid);
            c.drawLine(x0, bottom, x1, bottom, grid);
            c.drawText(fmt(hi), 4 * den, top + 10 * den, label);
            c.drawText(fmt(lo), 4 * den, bottom, label);
            if (timeAxis) {
                float zx = x0 + (0 - xa) / (xb - xa) * (x1 - x0);       // the step, t = 0
                if (zx > x0 && zx < x1) c.drawLine(zx, top, zx, bottom, grid);
                c.drawText(String.format(Locale.US, "%.1f s", xa), x0, y1, label);
                c.drawText(String.format(Locale.US, "%.1f s", xb), x1 - 34 * den, y1, label);
            }
            if (dashed != null) c.drawPath(path(x, dashed, xa, xb, lo, hi, x0, top, x1, bottom), dash);
            for (int i = 0; i < ys.length; i++) {
                line.setColor(colors[i]);
                c.drawPath(path(x, ys[i], xa, xb, lo, hi, x0, top, x1, bottom), line);
            }
        }

        // ---- path() -------------------------------------------------------------------------
        //      Turns arrays of numbers into a line path scaled to the panel.
        static Path path(float[] x, float[] y, float xa, float xb, float lo, float hi,
                         float x0, float top, float x1, float bottom) {
            Path p = new Path();
            for (int i = 0; i < x.length; i++) {
                float px = x0 + (x[i] - xa) / (xb - xa) * (x1 - x0);
                float py = bottom - (y[i] - lo) / (hi - lo) * (bottom - top);
                if (i == 0) p.moveTo(px, py); else p.lineTo(px, py);
            }
            return p;
        }

        // ---- fmt() --------------------------------------------------------------------------
        //      Formats an axis number (no decimals above 100).
        static String fmt(float v) { return Math.abs(v) >= 100 ? String.format(Locale.US, "%.0f", v) : String.format(Locale.US, "%.1f", v); }
    }
}
