ELECTRONICS BOX V5 - HONEYCOMB TRAY (Nov 25, 2026) - Ethan DiMeo
=================================================================
Reprint ONLY the tray. Your printed COLLAR v3.1 stays: the tray's top rim and the 4 corner tabs with
their M3 pilot holes are exactly where V3 / V4.1 had them. 4x M3 x 10 at the corners, same as before.
Frame (same as V3): +X = true RIGHT, +Y = true FRONT, z = 0 = underside of the plate.

WHY V5
  - The box was too tight.
  - The landing legs are off, so the drone now sits and lands on the bottom of the box.
  - The battery moved to the top of the frame, so nothing hangs under the box any more.

WHAT CHANGED FROM V4.1
  1. BIGGER. Below the collar joint the walls lean out at 45 deg (no supports needed) to a wider, longer body:
       outside 151 x 96 mm below the roof (V4.1: 130 x 72 at the kit, 169 x 86 at the right bay)
       12 mm deeper: bottom 98.1 mm under the plate (V4.1: 86.1)
       inside volume 979 cm3 (V4.1: 725, V3: 612) - about 35 % more room than V4.1
       18 mm of room each side of the kit's fence for wires (V4.1: 6 mm)
       40 mm free strip in front of the proto board, 8.8 mm behind its right end
     The right corners are cut at 45 deg so every point stays within 93.2 mm of the frame center
     (the prop discs start at 98 mm).
  2. FLAT BOTTOM. The battery strap ears under the floor are gone. The bottom is one flat face (all of it at
     z = -98.1) to land on. The floor is 2.4 mm thick.
  3. HONEYCOMB. 82 hexagon windows, 11 mm across the flats, with 3 mm ribs, in the long walls, the right
     end wall, the left end wall (above the USB ports) and the free parts of the floor (open area 86 cm2).
     You can reach in and route wires through them, it is lighter for its size, and it prints faster than
     solid walls of this size would.
       - Flat-top hexagons: the only overhang is the flat top of each one, a 6.4 mm bridge. No supports.
       - Every panel keeps a 3 mm solid frame. The 45 deg roof, the corners, the wire bays and the floor
         under the kit stay solid.
  4. Walls 2.4 mm (V4.1: 2.0). The two wire bays (front and back lobes) keep 2.0 mm so they match the collar.
  5. Kept from V4.1, unchanged:
       - PIC32 kit cradle (fence, crush ribs, M2.5 boss, J1 and foot pedestals) and the port windows in the
         left end wall (they moved down 12 mm with the floor, together with the kit)
       - 7805 proto board rails, corner lips, zip-tie slots and the USB window in the back wall
       - relay on edge in the back wire bay on 4 posts, at the SAME height as V4.1 (it did not move down),
         and the 11 mm U-slot for the battery leads with "BATT" under it
       - FRONT on the front wire bay

FIT CHECK (fit_check_v5.txt) - all 0 overlap:
  collar v3.1, the plate, the kit and its plugs, the J12 jumpers, the proto board, its parts and USB plug,
  the relay, its parts and screws, the 10 AWG leads and both bullets.
  Kit drops straight in. Proto board install path checked step by step (see below).
  STL is watertight. Overhang scan: only flat bridges (hexagon tops 6.4 mm, USB window 16 mm, port
  windows 29 mm as in V3/V4.1, left tab undersides as V3) - nothing that needs supports.

PRINT (Creality K2 Pro, standard PLA, 0.4 mm nozzle)
  - print/tray_v5.stl, FLOOR DOWN (as exported), supports OFF. 160 x 114 x 76 mm, fits the 300 x 300 bed.
  - 0.2 mm layers, 4 walls (the 2.4 mm walls print solid), 15 % infill, brim optional (3 mm helps if the
    long floor corners lift).
  - About 109 cm3 of plastic = about 136 g if it were 100 % solid; the slicer will show the real weight and
    time (the thin walls print solid, so expect most of that).
  - Optional first: print/test_relay_bay_v5.stl (about 8 cm3) = the back wire bay only. Screw the relay on
    and check the holes and that the + lead reaches out of the slot. Same bay as V4.1.

PUT IT TOGETHER (relay and proto board first, then the kit)
  1. Relay: 4x M3 x 8 or M3 x 10 into the 4 posts in the back wire bay, power end (NC, COM, NO) UP.
     Wire COM (+ lead to the battery bullet) and NO (to the plate's plus pad) first. Lay the + lead over the
     board's top edge into the slot.
  2. Proto board, BEFORE the kit: hold it about 14 mm left of and 13 mm in front of its spot, lower it
     through the top opening, slide it back 13 mm (the USB socket toward the back wall), then right 14 mm
     just above the kit's fence, and drop it the last 3 mm between the 4 corner lips. Zip-tie it.
  3. Kit into its cradle (port end at the left wall), M2.5 x 8 screw. J12 jumpers; relay CH1 to J12 pin 33.
  4. Lift the tray onto the collar: the minus lead from the plate drops into the slot. 4x M3 x 10 at the corners.
  5. Battery leads (from the battery on top) to the two male bullets at the back.
  6. USB cable from the proto board's socket (back wall window) to the kit: run it up the back wall,
     not under the box - the box bottom is the landing face now.

NOTES
  - The bullets hang behind the back wire bay. Keep + and - apart, heat shrink on both.
  - 7805: TO-220 heatsink (about 1.9 W). The floor and walls next to the proto board are honeycomb, so it gets air.
  - The 10 A relay fits the back bay (26 wide in 34). A 30 A flight relay is usually bigger: measure first.
  - Every size is a parameter at the top of src/tray_v5.py
    (run: python tray_v5.py OUTPUT_DIR; python fit_check_v5.py OUTPUT_DIR; python render_v5.py RENDER_DIR).
    The hexagon size is HEX_F / HEX_RIB, the depth is DEPTH_ADD.

FILES
  print/tray_v5.stl, print/test_relay_bay_v5.stl     step/tray_v5.step     renders/*.png
  fit_check_v5.txt
  src/tray_v5.py, fit_check_v5.py, render_v5.py
  src/box_v3.py, collar_v3_1.py, f450_bottom_plate.json (unchanged V3 sources it builds on)
