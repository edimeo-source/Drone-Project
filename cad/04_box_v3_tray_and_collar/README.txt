ELECTRONICS BOX V3 (Nov 19, 2026) - 2 printed parts: TRAY (red in the renders) + COLLAR (white)
==================================================================================================
Hangs under the F450 bottom plate like V2 (the plate is the lid). Replaces box V2.
Frame used in the model: +X = true RIGHT (red arms), +Y = true FRONT, z = 0 = underside of the plate.

WHAT CHANGED FROM V2 (from your 10 photos)
  1. PIC32 kit RIGHT SIDE UP and flat. V2 assumed J12's pins point down, which is why the kit had
     to go in upside down. Your photos show J12's long pins point UP (same side as the USB ports and
     the Ethernet board), so the kit now lies right side up and you plug J12 from above.
  2. Cradle ("slot for the back"): the board sits 5 mm above the floor. Everything on its underside
     drops into that space: J1 (Hirose FX10A-168P-SV1, 3.5 mm), the rubber foot (3M SJ61A8,
     9.5 x 3.4 mm), micro-USB J4 (3 mm). The board rests on the screw boss + a corner pad; J1 and the
     rubber foot each have a pedestal right under them so the board can't bend when you push jumpers
     onto J12. A fence on 3 sides with 6 crush ribs holds it snug (fits a 50.5 - 51.2 mm wide board).
     Screw: M2.5 x 8 self-tapping into the boss (same as V2).
  3. Ports: window A = USB-A J5 + one mini-USB + micro-USB J4 (under the board); window B = the other
     mini-USB. Sized for fat plug over-molds (USB-A 17 x 9, mini 12 x 9, micro 11 x 7.5).
     Ethernet: the board can stay on. Right side up the RJ45 points up and clears everything
     (it is not used, so you can also just unplug it).
  4. Legs: V2's long walls ran right under the 4 landing-gear brackets, so it hung 5 mm low.
     The COLLAR (the slotting bracket) is now 22 mm tall; its walls step in around each leg bracket
     and follow the leg as it curves out. The full-width TRAY only starts 22 mm under the plate.
  5. Wire bays: both parts are 38 mm wide out to y = +/-56 under the plate's front and back
     32 x 12 edge openings, so the ESC servo leads drop INTO the box, right next to J12.
  6. Two parts: the kit and the power board drop straight into the open tray, and if the legs need
     more room you reprint only the collar.
  7. 1.5 in (38.1 mm) deeper than the first V3 (your request, for more room): the tray is 64 mm tall.
     The kit and power board still sit on the tray floor, so the extra room is above them.
     XT60 window stays 27 mm above the box bottom (right end of the tray, just above the power board),
     so the battery lead reaches it like on V2. Vent slots at y = 6 - 21.

COLLAR v3.1 (Nov 20, 2026) - PRINT THIS ONE, NOT collar_v3.stl
  The V3 collar hit the leg brackets: they reach 9 mm further in than the photo guess. v3.1 traces
  them from your calipers + close-ups and wraps each one with a 1 mm gap. Nothing else changed:
  same 4 corner tabs / screw holes and rim as the tray you printed, same keys and bolts into the frame.
    bracket: 23.2 long (nose to outer edge), 28.5 wide, 7.3 thick (calipers), + 3 mm screw heads
    nose 18.5 mm in from the 2 arm screws, 10.6 mm wide, chamfers out to full width 10.5 mm in
    leg: 14.9 wide (calipers), lands on the outer half of the bracket, the thin rib runs to the nose
  Each pocket goes straight down to a 2 mm floor at the joint face, so it prints with no supports
  (the floors are on the bed) and the box stays closed. The floor is left open where the leg leaves.
  Fit check (fit_check_collar_v3_1.txt): 0 overlap with the measured legs, the plate and the tray;
  the old collar overlapped the measured legs by 1616 mm3, which is why it wouldn't go on.
  Print: print/collar_v3_1.stl, same K2 Pro settings below, about 1.5 h, ~45 g. Brim optional now.

SIZES
  Tray    148 x 112 x 64.1 mm (z -86.1 .. -22), 89 cm3     Collar 172 x 112 x 23.4 mm (z -22 .. 0 + keys), 38 cm3
  Inside: 61.7 mm from the tray floor to the joint, 83.7 mm from the floor to the plate.
  Box bottom 86.1 mm under the plate; battery bottom 112.6 mm under the plate (V2: 74.5).
    -> stand the drone on its legs and check the plate underside is more than ~125 mm off the table.
  About 145 g of PLA for both (tray ~105 g, collar ~45 g). Fit check (fit_check_v3.txt): 0 overlap with the kit, its ports,
  plugs, J12 jumpers, power board, XT60, battery, plate and both leg models; kit and power board
  drop straight in.

HARDWARE
  8x M3 x 10   4 + nuts: box to the frame through the tab slots (nut traps under the end plates)
               4 without nuts: collar to tray at the 4 corners (they thread into 2.6 mm holes; snug, don't crank)
  1x M2.5 x 8 self-tapping   PIC32 to the boss
  2x zip ties (max 5 mm), 2x 20 mm velcro straps (same as V2)

PRINT ORDER (cheap tests first)
  1. print/test_cradle_v3.stl (about 30 min): the cradle only. Drop the kit in, check the fit, screw it.
  2. print/collar_v3.stl: hold it up under the frame WITH THE LEGS ON. Keys must go into the 4 slots
     and the rim must touch the plate everywhere. If a leg touches, tell me where (see MEASURE).
  3. print/tray_v3.stl. (Once 1 and 2 check out, tray + collar fit on one K2 Pro plate together.)
  Orientation: as exported. Tray floor down, collar joint face (the flat ring with 4 corner tabs) down.

CREALITY K2 PRO + STANDARD PLA (Creality Print, K2 Pro 0.4 mm nozzle profile, "Generic PLA")
  Nozzle 215 C (first layer 220)     Bed 55 C (textured PEI)
  Chamber: leave the PLA preset alone, no chamber heating (PLA softens and can jam the extruder)
  Layer 0.20 mm (first layer 0.20), line width 0.42
  Walls 4 (the 2 mm walls come out solid), top 5 layers, bottom 4 layers, infill 20 % gyroid
  Max volumetric speed 15 mm3/s  <- the important one: standard PLA can't melt as fast as Hyper PLA,
     this caps every speed automatically
  Speeds: outer wall 120, inner wall 200, infill 250, first layer 50, bridges 40 mm/s
  Outer-wall acceleration 5000 mm/s2 (cleaner holes and keys), rest default
  Fan 100 % from layer 3, bridge fan 100 %
  Supports OFF. Only short bridges: window tops (29 and 14 mm), XT60 window (17 mm),
     4 small triangle roofs inside the collar corners (~10 mm), nut traps.
  Brim: collar 5 mm (its first layer is just the 2 mm wall ring + tabs), tray none
  Elephant-foot compensation 0.15 mm (keeps the joint face and the cradle true)
  Seam: aligned / back. Run the printer's auto calibration first; dry the PLA if it's old.
  Heat: PLA gets soft around 55-60 C. Keep the 7805 heat sink off the walls (the vents are next to it).

ASSEMBLY
  1. Kit into the tray right side up, USB end against the 2 windows; press it down between the ribs.
     M2.5 x 8 into the boss. (If the screwdriver can't get past the Ethernet board, unplug that board,
     screw, plug it back.)
  2. Plug the J12 jumpers from above: ESC leads, the 7-wire sensor cable, power board leads (pins 2, 33, 34).
     The kit sits 38 mm lower than in the first V3 (J12 pin tips ~69 mm under the plate), so the ESC
     leads and the 7-wire sensor cable to the deck need about 40 mm more length.
  3. Power / kill board flat in the right bay, 2 zip ties through the floor slots.
  4. Collar on, 4x M3 x 10 down through its corner tabs into the tray.
  5. XT60 into the collar's right-end window.
  6. ESC leads up through the front / back bays and the plate's edge openings; sensor cable up to the deck.
  7. Box up under the plate: keys into the 4 tab slots, 4x M3 x 10 + nuts.
  8. Battery under the floor, straps through the 4 ears, plug into the XT60.

HOW ACCURATE IT IS
  Datasheets: J1 height (FX10A-168P-SV1 = 5 mm stack), rubber foot (kit BOM: SJ61A8 .375 x .135 in),
    USB parts (kit BOM). Board 71.5 x 51.0: checked on your underside photo with J12's 2.54 mm pin
    pitch as the ruler (about 50.5 wide, +/- 0.5) - the crush ribs cover that.
  Traced from your photos (+/- 1-2 mm): leg brackets and leg curve, port positions, J1 / foot / J4
    positions. The box keeps 1.5 mm away from a deliberately oversized leg (bracket 36 x 18 x 9 mm,
    leg 18 wide, going out only 1 mm per mm down - the photos show ~1.4), and the windows are oversized.

MEASURE (only if the collar test-fit fails; then change src/box_v3.py and reprint the collar)
  - Leg bracket under the plate: its nearest corner must be >= 2 mm from the short end of the
    gold-rimmed front / back edge opening (photos: ~4 mm)            -> LEG_FLANGE_L
  - Bracket + its nuts must hang <= 9 mm under the plate             -> LEG_FLANGE_T
  - Power board 45 x 58 x 22 (same as V2)                            -> PWR_L / PWR_W / PWR_H
  Still open (not in the box yet, same as V2): the relay module (and the 30 A one for flight).

FILES
  print/  tray_v3.stl, collar_v3_1.stl (use this), collar_v3.stl (old), test_cradle_v3.stl (mm, oriented for printing)
  step/   tray_v3.step, collar_v3.step, assembly_v3.step (with plate, leg estimate, kit, power board, battery)
  renders/ 1-8 png
  src/    box_v3.py (all parameters at the top), collar_v3_1.py (leg pockets), render_v3.py, render_collar_v3_1.py, f450_bottom_plate.json
  fit_check_v3.txt
