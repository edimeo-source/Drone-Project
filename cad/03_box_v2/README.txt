ELECTRONICS BOX V2 (Nov 18, 2026) - PIC32 kit + power / kill board, battery underneath
==========================================================================================
Hangs under the F450 bottom plate (the plate is its lid). The IMU and the Bluefruit are on the
sensor deck v3 now, so they are not in here. Replaces box rev 1.

WHAT CHANGED FROM REV 1
  - Bluefruit slot gone; side windows, floor cutout and the 4 PWM wall slots gone (walls closed).
    Every wire leaves through the open top and the bottom plate's big FRONT and REAR center
    openings: sensor cable, ESC signal leads, power leads to the PDB.
  - 6 mm longer (inside 126) and 4 mm wider (inside 68) for wire slack: plug-in leads only,
    nothing cut or soldered. 8.5 mm beside the kit on each side, 14 mm over it.
  - 4 pockets in the top edge for the arm screw heads under the plate at (+/-46.5, +/-32).
    REV 1 SAT ON THOSE HEADS (99 mm3 overlap in the model) - don't print rev 1.
  - Power bay at the front for the 45 x 58 proto board (7805 board), flat on the floor, 2 zip ties.
    XT60 window moved up, above the board. 4 vent slots in the front wall next to the 7805.

OPENINGS AND WHAT THEY ARE FOR
  rear window ......... USB: J3 program, J11 PuTTY / tuner, J4 power (under the board)
  front window ........ XT60 (battery plugs in from outside)
  4 vent slots ........ front wall, lets the 7805 breathe
  4 floor slots ....... 2 zip ties around the power board (they run under the floor, over the battery)
  4 strap ear slots ... 2 velcro battery straps
  4 top-edge pockets .. arm screw heads

PRINT (K1 SE, Hyper PLA)
  print/box_v2.stl floor down, as exported. NO supports. 0.2 mm, 3 walls, 15% infill. ~60 g.
  Only unsupported spans are bridges: under the end plates between the 45 deg gussets (21 mm),
  USB window top (45 mm), XT60 window top (17 mm), vent tops (3 mm), nut traps.

HARDWARE
  4x M3 x 10 + nuts        box to the frame (down through the tab slots, nuts in the traps)
  1x M2.5 x 8 self-tapping PIC32 to the center post
  2x zip ties (max 5 mm)   power board
  2x 20 mm velcro straps   battery

ASSEMBLY
  1. Take the Ethernet daughter board off the kit.
  2. Plug the J12 jumpers onto the kit's underside (ESC leads, the 7-wire sensor cable, power).
  3. Kit in, USB edge to the rear window, screw it to the post.
  4. Power board in the front bay, zip-tied; XT60 through the front window.
  5. Feed the sensor cable, ESC leads and power leads up through the plate's front / rear
     center openings (wires can run over the top of the kit, 14 mm).
  6. Box up under the plate: keys into the 4 tab slots, 4x M3.
  7. Battery under the floor, straps through the 4 ears, plug into the XT60.

MEASURE FIRST (src/box_v2.py)
  PWR_L x PWR_W x PWR_H = 45 x 58 x 22   your proto board and its tallest part
  KIT_L x KIT_W = 71.5 x 51.0, HOLE_FROM_J5_EDGE = 25.8
  Landing gear: the battery hangs 74.5 mm under the plate.
