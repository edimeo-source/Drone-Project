// =============================================================================================
//  ELC275 DRONE PROJECT  -  Ethan DiMeo
//  File    : tools/android_drone_tuner/v3.1_motor_adjust_2026-11-25_CURRENT/test/AppTest.java
//  What    : PC (JVM) test of the 3.1 app's run parser, 4-motor mixer and turn buttons, on REAL firmware 3.1 output
//  Date    : 2026-11-25
//  Run     : sh run_test.sh (after the v3.1 host_tests/run_tests.sh)
//  METHODS: check(), main()
//  Every method below has a '// ---- name()' title line with what it does.

// JVM test of the phone app's run parsing, 4-motor mixer and motor-adjust buttons against REAL firmware output
// (files written by host_tests/run_tests.sh into tuner_tests/fw_out). No phone needed.
package edu.dtcc.elc275.tuner;   // same package: tests the app's own (package-private) methods
import java.nio.file.*;
import java.util.*;

public class AppTest {
    static int fails = 0;
    // ---- check() ----------------------------------------------------------------------------
    //      Prints PASS or FAIL for one check.
    static void check(boolean c, String m) { System.out.println((c ? "PASS  " : "FAIL  ") + m); if (!c) fails++; }

    // ---- main() -----------------------------------------------------------------------------
    //      Runs the checks on the firmware's own output: Both-mode Bluetooth run, the mixer vs
    //      the firmware's 4 ESC pulses, one-axis lines, and the turn buttons.
    public static void main(String[] a) throws Exception {
        Path dir = Paths.get(a[0]);
        // 1. BOTH-mode Bluetooth run: RUN_BEGIN field 10 = 3, RD quads
        List<String> lines = Files.readAllLines(dir.resolve("phone_both.txt"));
        int total = 0, axis = 0, got = 0;
        float[] ang = null, u = null, r = null, p = null, rc = null, pc = null;
        for (String s : lines) {
            String[] f = s.split(",");
            if (f[0].equals("RUN_BEGIN")) {
                total = Integer.parseInt(f[1]);
                axis = f.length >= 11 ? Integer.parseInt(f[10]) : 2;
                ang = new float[total]; u = new float[total]; r = new float[total];
                p = new float[total]; rc = new float[total]; pc = new float[total];
            } else if (f[0].equals("RD")) {
                got += MainActivity.parseRd(f, axis, total, ang, u, r, p, rc, pc);
            }
        }
        check(axis == 3 && total == 58 && got == 58, "BOTH run: axis 3 read from RUN_BEGIN, all 58 samples");
        // expected: the numbers in the RD text in order (sim_ble already proves they equal the firmware arrays)
        List<Integer> nums = new ArrayList<>();
        for (String s : lines) if (s.startsWith("RD,")) {
            String[] f = s.split(",");
            for (int i = 2; i < f.length; i++) nums.add(Integer.parseInt(f[i]));
        }
        boolean ok = nums.size() == 4 * 58;
        for (int k = 0; ok && k < 58; k++)
            ok = r[k] == nums.get(4 * k) / 10f && p[k] == nums.get(4 * k + 1) / 10f
                    && rc[k] == nums.get(4 * k + 2) && pc[k] == nums.get(4 * k + 3) && ang[k] == p[k] && u[k] == pc[k];
        boolean shape = r[5] > 3.0f && Math.abs(p[0] + 2.1f) < 0.051f && rc[0] == 12 && pc[57] == 31;   // the sim's test signals
        check(ok && shape, "BOTH run: roll, pitch, RC, PC of every sample in the right place (graph = pitch, u = PC)");

        // 2. the phone's mixer vs the firmware's motor outputs (PC dump, BOTH mode: m1..m4, RC, PC)
        int n = 0, bad = 0;
        for (String s : Files.readAllLines(dir.resolve("pc_both.txt"))) {
            String[] f = s.split(",");
            if (!f[0].equals("D") || f.length < 19) continue;
            float m1 = Float.parseFloat(f[13]), m2 = Float.parseFloat(f[14]), m3 = Float.parseFloat(f[15]), m4 = Float.parseFloat(f[16]);
            float RC = Float.parseFloat(f[17]), PC = Float.parseFloat(f[18]);
            if (Math.min(Math.min(m1, m2), Math.min(m3, m4)) <= 1100.5f || Math.max(Math.max(m1, m2), Math.max(m3, m4)) >= 1999.5f) continue;
            float T = (m1 + m2 + m3 + m4) / 4;
            float[][] m = MainActivity.motors4(T, new float[]{RC}, new float[]{PC}, 1);
            if (Math.abs(m[0][0] - m1) > 1.01f || Math.abs(m[1][0] - m2) > 1.01f || Math.abs(m[2][0] - m3) > 1.01f || Math.abs(m[3][0] - m4) > 1.01f) bad++;
            n++;
        }
        check(n > 20 && bad == 0, "phone mixer FL/FR/RL/RR = firmware's 4 ESC pulses on " + n + " real samples (within 1 us rounding)");

        // 3. one-axis line (pairs) still parses
        float[] A = new float[8], U = new float[8];
        int g = MainActivity.parseRd("RD,0,10,40,20,35,30,30,40,25".split(","), 1, 8, A, U, null, null, null, null);
        check(g == 4 && A[1] == 2.0f && U[1] == 35f, "one-axis RD line: angle / correction pairs");
        // 4. app 3.1 motor adjust: the turn-CW button and the command it sends
        int[] o = new int[5];
        MainActivity.yawStep(o, 10);
        check(MainActivity.offsetCommand(o).equals("O,10,-10,-10,10"),
              "turn CW (step 10) sends O,10,-10,-10,10 (the command the firmware test flies with)");
        check(o[1] + o[3] == 0 && o[1] + o[2] == 0, "turn CW keeps the left pair and the front pair sums: roll and pitch unchanged");
        for (int i = 0; i < 30; i++) MainActivity.yawStep(o, -25);
        check(o[1] == -200 && o[2] == 200 && o[3] == 200 && o[4] == -200, "turn CCW many times stops at the firmware's +/-200 us");
        System.out.println(fails == 0 ? "\nall passed" : "\n" + fails + " failure(s)");
        System.exit(fails == 0 ? 0 : 1);
    }
}
