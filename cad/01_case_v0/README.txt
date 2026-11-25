>>> NOV 19: CURRENT BOX = box_v3/ (tray + collar, kit right side up, clears the landing gear). See box_v3/README.txt.
    box_v2/, box_rev1/ (sits on the arm screw heads - do not print), imu_adapter/ (v2) and case.stl below are superseded. <<<

ELC275 DRONE - ELECTRONICS CASE + IMU MOUNT  (Nov 18, 2026)
==============================================================

FILES
  print/case.stl, print/imu_mount.stl      slice these
  step/case.step, step/imu_mount.step      open in Fusion 360 to edit
  step/assembly.step                       everything in place: F450 bottom plate (real slot layout),
                                           simplified top plate, case, IMU mount, PIC32 kit, Bluefruit,
                                           battery, IMU (plain blocks)
  renders/*.png                            previews
  fit_check.txt                            clash check results (all 0)
  src/case.py                              CadQuery source; every size is in the PARAMETERS block
  src/f450_bottom_plate.json               bottom plate outline + slots (sliced from an open F450 model)

COORDINATES  origin = frame center, +X = FRONT (red arms), +Y = left, +Z = up,
             z = 0 = underside of the F450 bottom plate.

WHERE EVERYTHING GOES (weight centered on the frame's middle)
  IMU ............ middle of the bottom plate, between the plates, on the slot adapter
                   (imu_adapter/ folder, keys into the 3 center slots so it sits square).
                   Board X arrow to the FRONT, Y arrow to the left. Wires go down through a
                   bottom-plate slot into the case. (print/imu_mount.stl, the old top-plate
                   mount, is no longer used.)
  Case ........... hangs under the bottom plate, centered. 4 keys drop into the 4 tab slots
                   (29.8 x 4 mm at x = +/-70, y = +/-17) and 4 M3 bolts go through the same slots.
  PIC32 kit ...... in the case, top side UP (LEDs/USB up), on 4 corner towers 30 mm high, because
                   J12 is on the kit's UNDERSIDE (kit user's guide sec. 2) and the jumpers hang down.
                   USB edge to the REAR (rear window: J3 program, J11 PuTTY/tuner, J4 power).
  Bluefruit ...... front of the case, on the taped pad.
  Battery ........ under the case floor, centered between 2 rails, 2 velcro straps through the 4 ears.
                   Fits the Zeee "shorty" 75 x 34 x 26.5 and the regular 102 x 34 x 23.
  ESC signal wires come down through the bottom plate's rounded slots (x = +/-46) into the case.

PRINT (Creality K1 SE, Hyper PLA, 0.4 nozzle)
  case.stl ....... floor DOWN on the bed (as exported). No supports (45 deg gussets under the end
                   plates). 0.2 mm layers, 3 walls, 15% gyroid. About 40 g. The side windows bridge
                   50 mm: a little sag on the window top is fine.
  imu_mount.stl .. flat side down. No supports. 0.2 mm, 100% or 4 walls. About 9 g.

HARDWARE
  4x M3 x 10 bolts + 4x M3 nuts      case to the frame (bolt from above through the tab slot,
                                      nut sits in the hex pocket under the end plate)
  IMU: see imu_adapter/README.txt (2x M2.5 x 8 + nuts, 4x M2.5 x 4-5 self-tapping)
  1x small zip tie                   across the PIC32 board through the two wall holes (holds it down)
  2x 20 mm velcro battery straps
  Foam double-sided tape             Bluefruit to its pad

ASSEMBLY ORDER
  1. Unplug the kit's Ethernet daughter board (not used; saves height and ~10 g).
  2. Plug the J12 jumpers onto the kit's underside first, then drop the kit into the towers,
     USB edge to the rear window. Zip tie over the board through the wall holes.
  3. Tape the Bluefruit onto the front pad. Route all jumpers out the top of the case.
  4. Feed the ESC signal/ground wires and the IMU wires down through the bottom plate's
     rounded slots, plug them onto the J12 jumpers per J12_PINOUT.html.
  5. Lift the case up under the plate: the 4 keys drop into the 4 tab slots. Bolt it (4x M3).
  6. Strap the battery under the case. XT60 lead goes up the side to the frame's power pads.
  7. IMU: see imu_adapter/README.txt (adapter keys into the 3 center slots, IMU on top).

MEASURE FIRST (then change the number in src/case.py and run it again)
  - PIC32 kit outline (KIT_L x KIT_W = 71.5 x 51.0 from Microchip drawings, +/-1 mm)
  - where the kit's USB connectors and switches are (rear window and side windows are generous)
  - tab slot positions on YOUR bottom plate (SLOT_X = 70.1, SLOT_Y = 17.0)
  - IMU board hole spacing (IMU_HOLE_SP = 20.32) and how far its header pins stick down
  - top plate: is there a solid area at the center for the tape (photo shows a center hole)

REGENERATE
  pip install cadquery==2.8.0 vtk
  python src/case.py out          (STEP/STL + fit check)
  python src/render_vtk.py out/renders
