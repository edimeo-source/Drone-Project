# Docs

| File | What |
|---|---|
| [`PINOUT.md`](PINOUT.md) | J12 pinout now, and for every version. |
| [`wiring/complete_wiring.png`](wiring/complete_wiring.png) | Whole-drone wiring diagram (also PDF). |
| [`wiring/actual_wiring.png`](wiring/actual_wiring.png) | Real-parts wiring of the power/relay board (Nov 19). |
| [`wiring/power_kill_wiring.png`](wiring/power_kill_wiring.png) | Power + relay kill schematic, top view and J12 table. |
| [`wiring/src/`](wiring/src) | Python that draws those diagrams. |
| [`pinout_sheets/`](pinout_sheets) | The printable J12 pinout pages (seesaw, Nov 15, and drone, Nov 18). Download and open in a browser. |
| [`history/PINOUT_AND_PLAN_2026-11-15.md`](history/PINOUT_AND_PLAN_2026-11-15.md) | The Nov 15 pinout and build plan (why the IMU CS moved to pin 16). |
| [`history/README_FOR_INTEGRATION_2026-11-20.md`](history/README_FOR_INTEGRATION_2026-11-20.md) | Notes given to the partner for the Bluetooth merge (Nov 20). |

Power/relay notes (Nov 19):

- The relay contacts are rated 10 A.
- Hover is about $4\times 4.4\ \text{A}=17.6\ \text{A}$. Fly on a 30 A (or larger) relay module. The 10 A one is for low-throttle bench tests only.
- The battery now plugs in with separate bullet leads (no XT60, Nov 24).
