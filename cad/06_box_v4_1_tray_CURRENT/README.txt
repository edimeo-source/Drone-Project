ELECTRONICS BOX V4.1 - NEW TRAY ONLY (Nov 24, 2026) - Ethan DiMeo
===================================================================
Reprint ONLY the tray. Your printed COLLAR v3.1 stays: the tray's rim, corner tabs and screw holes
are exactly V3's. Frame (same as V3): +X = true RIGHT, +Y = true FRONT, z = 0 = underside of the plate.

V4.1 = V4 + THE BATTERY CONNECTION
  The battery has separate FEMALE bullets (+ and -). On the drone:
    (-) male bullet on a 2 - 2.5 in (51 - 64 mm) lead from the plate's minus pad at the BACK
    (+) male bullet on a 1.5 in (38 mm) lead from the relay's COM terminal
  Both leads must reach outside the box, so the relay now sits right under the battery pads.

WHAT CHANGED FROM V4
  1. Relay moved out of the right bay into the BACK wire bay (the bay under the plate's back edge
     opening, where the battery pads are). It stands on edge on 4 sideways posts on that bay's back wall:
       - power end (NC, COM, NO) on TOP, 25 mm under the plate. NO goes straight up to the plate's
         plus pad; COM's + lead goes back over the board's top edge and out the slot.
       - control end (CH1, VREF, DC-, DC+) at the bottom, 9 mm above the floor, next to J12.
       - board 6 mm off the wall (the relay's pins are about 2 mm). Posts have 45 deg wedges under
         them, so they print with no supports.
  2. U-slot in that bay's back wall, centered behind the relay's top terminals: 11 mm wide, open at
     the top, 14 mm deep. Both leads drop in from above, and their bullets hang outside at the true
     back of the drone, where you plug the battery in. "BATT" is engraved below it.
       + lead path: about 18 mm inside the box, so a 38 mm lead leaves about 20 mm outside.
       - lead: comes down from the plate's back edge opening into the same slot (22 - 36 mm below the
         plate). A 51 - 64 mm lead reaches.
     When you lift the tray onto the collar, the minus lead hanging from the plate drops into the
     open top of the slot.
  3. XT60 window removed (the battery uses bullets now). Tell me if you still want it.
  4. The relay's floor posts in the right bay are gone. The front half of the right bay is now free
     space (wire slack, 7805 heatsink air).
  5. Everything else is V4:
       - right bay size and 45 deg roof
       - proto board rails, lips and zip slots
       - USB window in the back wall
       - vents, kit cradle, port windows, strap ears

FIT CHECK (fit_check_v4_1.txt) - all 0 overlap:
  collar v3.1, the caliper-measured legs, the plate, the kit and its plugs, the J12 jumpers,
  the battery, the proto board and its USB plug, the relay and its parts, the relay screws,
  the 10 AWG leads, both bullets.
  Farthest point from the frame center: 93.4 mm (the prop discs start at 98).

PRINT ORDER
  1. print/test_relay_bay_v4_1.stl (about 12 g, 44 x 24 x 56 mm). This is the back bay only:
     posts, wedges and the slot. Screw the relay on and check:
       - the holes line up;
       - the + lead reaches out the slot with the bullet outside.
  2. print/tray_v4_1.stl (about 119 g). Floor down, supports OFF, same K2 Pro PLA profile as V3.

PUT IT TOGETHER (relay and proto board first, then the kit)
  1. Relay: screw it to the 4 posts in the back bay with 4x M3 x 8 or M3 x 10, power end UP.
     Small pads on the outside of the back wall stop the screw tips 1.1 mm short of poking through.
  2. Wire COM (+ lead) and NO (to the plate's plus pad) before you close it up. Lay the + lead over
     the board's top edge into the slot.
  3. Proto board into the right bay, as in V4 (slide it right under the roof, zip-tie it).
  4. Kit into its cradle, M2.5 x 8 screw. J12 jumpers; relay CH1 to J12 pin 33.
  5. Lift the tray onto the collar: the minus lead from the plate drops into the slot.
     4x M3 x 10 at the corners.
  6. Plug the battery's female bullets onto the two male bullets at the back. Strap the battery.

NOTES
  - The bullets hang in the open behind the back bay. Keep the + and - bullets apart and cover
    them with heat shrink, so a bare + bullet can't touch the frame.
  - 7805: TO-220 heatsink (about 1.9 W), kept 5 mm from the PLA.
  - The back bay fits the 10 A RM1HLE (26 wide in a 34 mm bay). A 30 A flight relay is usually
    bigger: measure it before you buy.
  - Every size is a parameter at the top of src/tray_v4_1.py
    (run: python tray_v4_1.py OUTPUT_DIR; render_v4_1.py RENDER_DIR for the pictures).

FILES
  print/tray_v4_1.stl, print/test_relay_bay_v4_1.stl     step/tray_v4_1.step     renders/*.png
  fit_check_v4_1.txt     src/tray_v4_1.py, render_v4_1.py
  src/box_v3.py, collar_v3_1.py, f450_bottom_plate.json (unchanged V3 sources it builds on)
