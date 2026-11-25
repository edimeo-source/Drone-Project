ELECTRONICS BOX V4 - NEW TRAY ONLY (Nov 24, 2026) - Ethan DiMeo
=================================================================
Reprint ONLY the tray. Your printed COLLAR v3.1 stays: the tray's top rim, the 4 corner tabs and
their screw holes are exactly V3's, so the collar bolts on unchanged.
Frame used in the model (same as V3): +X = true RIGHT, +Y = true FRONT, z = 0 = underside of the plate.

WHY
  The 7805 proto board and the relay module did not both fit in V3's 45 x 58 power bay.
  Sizes used (from your caliper photos + the parts sheet):
    7805 proto board (cut DFRobot perfboard)   58.3 x 46.2 mm
      USB-A socket on one long edge, center 14.4 mm from the board's end, sticks out 4.8 mm
    Relay module RM1HLE                         50 x 26 mm, about 17 mm tall (+ about 2 mm of pins under it)
      4 holes, 3.1 mm, on 44.5 x 20.5 mm (your caliper read 44.5 on the long side)

WHAT CHANGED FROM V3
  1. Right bay is longer and wider: end wall at x = 84.5 (V3: 65), long walls at y = +/-42.75 (V3: +/-36).
     Inside: 68 x 81.5 mm from the kit's cradle fence to the end wall.
     Farthest point of the tray from the frame center: 93.4 mm (the prop discs start at 98).
  2. Above the extension the tray closes itself with a 45 deg roof that rises to the collar joint.
     It prints with no supports. Inside, the roof is 41 mm above the floor at the end wall.
  3. Proto board, back half of the bay, USB socket toward the BACK wall:
       - 3 mm rails under its two short ends (clears the solder joints)
       - 4 corner lips that locate the PCB edges
       - 2 zip-tie pairs across it, at x = 24 and 70.8
       - window in the back wall (20 x 15 mm) in front of the USB socket. The USB-A plug goes in from
         OUTSIDE, so the cable runs outside the box to the kit's mini-USB at the left end
         (about a 30 cm USB-A to mini-B cable).
  4. Relay, front half of the bay, on 4 posts (8 mm tall, 2.6 mm pilot holes for M3 screws).
     The control end (CH1, VREF, DC-, DC+) faces J12. The power end (NC, COM, NO) faces the end wall,
     with 13.5 mm of room for the thick battery wires.
  5. XT60 window: same side position (y = -10) and height (27 above the bottom) as V3, now in the
     new end wall. The battery lead needs about 20 mm more reach than on V3. Check it before printing.
  6. Vents: 4 in the end wall and 4 in the back wall, next to the proto board (7805 heat).
  7. Battery strap ears: left pair as V3, right pair moved out to the wider walls.
  8. Kit cradle, port windows, wire bays and FRONT text: unchanged from V3.

FIT CHECK (fit_check_v4.txt) - all 0 overlap:
  collar v3.1, the caliper-measured legs, the plate, the kit and its plugs, the J12 jumpers,
  the battery, the proto board and its parts, the USB plug, the relay and its parts,
  the relay screws, the XT60.
  The joint face is the same as V3: the collar covers 1318 mm2 of the rim (1321 on V3).

PRINT ORDER (cheap test first)
  1. print/test_bay_v4.stl (about 27 g). This is only the right bay floor with 12 mm of wall:
     rails, corner lips, relay posts, zip slots, bottom of the USB window.
     Set the real proto board and relay on it:
       - the relay screws must line up with the posts;
       - the proto board must sit between the 4 lips;
       - the USB socket must line up with the window.
  2. print/tray_v4.stl (about 119 g; V3 was about 105 g).
  Orientation: as exported, floor down. Settings: the same K2 Pro / standard PLA profile as V3
  (see below). Supports OFF.

PUT IT TOGETHER (boards first, then the kit)
  1. Relay: hold it level over the bay, about 7 mm left of its spot. Lower it, slide it right under
     the roof, and set it on the 4 posts. Screw in 4x M3 x 8 (or M3 x 10: the pilot holes leave
     1.4 mm of floor, so nothing pokes out at the battery).
     The 2 right-hand posts sit under the roof edge: tilt a long screwdriver about 8 deg to the left.
  2. Proto board: same move. Lower it about 13 mm left of its spot, slide it right until the USB
     socket faces the back-wall window, then drop it between the 4 corner lips. Strap it with
     2 zip ties (max 5 mm wide).
  3. PIC32 kit into its cradle, M2.5 x 8 screw (same as V3).
  4. XT60 into the end-wall window (same as V3). Wire it to the relay's COM / NO.
  5. J12 jumpers. Plug the USB cable in from outside: back-wall window to the kit's mini-USB.
  6. Collar v3.1 onto the tray with 4x M3 x 10, then bolt it to the frame as before.

HARDWARE (new for V4)
  4x M3 x 8 (or x 10) screws for the relay, 2x zip ties for the proto board,
  1x USB-A to mini-B cable about 30 cm. Everything else is reused from V3.

NOTES
  - 7805: the parts sheet puts it at about 1.9 W. Use a TO-220 heatsink and keep it 5 mm from the
    PLA walls.
  - This bay fits the 10 A RM1HLE (50 x 26). A 30 A relay for flight is usually wider: measure it
    before you buy, and the posts can be moved (src/tray_v4.py, RL_* and HOLE_* at the top).
  - Every size above is a parameter at the top of src/tray_v4.py
    (run: python tray_v4.py OUTPUT_DIR; render_v4.py RENDER_DIR for the pictures).

CREALITY K2 PRO + STANDARD PLA (same as V3)
  Nozzle 215 C (first layer 220), bed 55 C, no chamber heating.
  0.20 mm layers, 4 walls, 5 top / 4 bottom, 20 % gyroid.
  Max volumetric speed 15 mm3/s. Outer wall 120, inner wall 200, infill 250, bridges 40 mm/s.
  Fan 100 % from layer 3. Supports OFF. Elephant-foot compensation 0.15 mm. No brim needed.

FILES
  print/tray_v4.stl, print/test_bay_v4.stl     step/tray_v4.step     renders/*.png
  fit_check_v4.txt     src/tray_v4.py (V4 tray), render_v4.py
  src/box_v3.py, collar_v3_1.py, f450_bottom_plate.json (unchanged V3 sources it builds on)
