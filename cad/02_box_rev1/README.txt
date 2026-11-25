ELC275 DRONE - ELECTRONICS BOX REV 1  (Nov 18, 2026)
=====================================================
Holds the PIC32MZ EF Starter Kit, the Bluefruit LE SPI Friend and the power / Bluetooth-kill
board. Hangs under the F450 bottom plate (the plate is its lid). Battery strapped underneath.
The IMU is NOT in the box: it stays on the slot adapter in the middle of the bottom plate.
Replaces the earlier case (cad/print/case.stl).

FILES
  print/box_rev1.stl        slice this (already floor-down)
  step/box_rev1.step        open in Fusion 360 to edit
  step/assembly_rev1.step   box + bottom plate + PIC32 + Bluefruit + battery + IMU adapter/IMU
                            + an ORANGE PLACEHOLDER block for the power/kill board
  renders/*.png             previews
  fit_check_rev1.txt        clash check (all 0)
  src/box_rev1.py           CadQuery source, every size in the PARAMETERS block

SIZE   124 x 68 x 48 mm box (172 mm long with the end plates), inside 120 x 64 x 45.6.
       Lowest point (battery) 74.5 mm under the bottom plate. About 56 g PLA.

LAYOUT (+X = front / red arms)
  Rear ....... PIC32 kit, top side up, 30 mm off the floor (J12 is on its UNDERSIDE, the
               jumpers hang down). USB edge against the rear window: J3 program, J11 PuTTY /
               tuner, J4 power (underneath).
               Held by 1 x M2.5 x 8 screw through the kit's mounting hole into the center post,
               resting on the post + 4 side supports, 4 side lips stop it turning.
  Front bay .. power / kill board (zip ties) + XT60 window in the front wall.
               Bluefruit in a card slot on the LEFT wall: slides down, header edge up.
  Long walls . 4 U-slots for the ESC PWM cables, open at the top (the frame plate closes them):
                 left wall:  front slot = ESC 1 (FL),  rear slot = ESC 3 (RL)
                 right wall: front slot = ESC 2 (FR),  rear slot = ESC 4 (RR)
               Side windows under the kit: reach the J12 jumpers.
  Up to frame  IMU wires and the power leads to the PDB go up through the bottom plate's
               big opening right above the front bay (x 40-52).

PRINT (K1 SE, Hyper PLA)
  Floor down, as exported. NO supports. 0.2 mm layers, 3 walls, 15% infill.
  Checked on the mesh: the only unsupported spans are bridges - under the end plates
  between the 45 deg gussets (max 23 mm), USB window top (45 mm), side window tops (30 mm),
  XT60 window top (17 mm), nut trap ceilings.

HARDWARE
  4x M3 x 10 + 4x M3 nuts     box to the frame (bolt down through each tab slot, nut in the
                              hex pocket under the end plate)
  1x M2.5 x 8 self-tapping    PIC32 kit to the center post (2.2 mm pilot)
  2x zip ties (max 5 mm)      power/kill board (down through the floor slots, under the floor,
                              over the battery)
  2x 20 mm velcro straps      battery

ASSEMBLY ORDER
  1. Take the Ethernet daughter board off the kit (it won't fit under the plate).
  2. Plug the J12 jumpers onto the kit's underside.
  3. Kit in, USB edge to the rear window, screw it to the post.
  4. Bluefruit down into the card slot on the left wall, header edge up, pins toward the
     middle of the box.
  5. Power/kill board in the front bay, zip-tied. XT60 male through the front-wall window.
  6. ESC PWM cables into the wall slots (see above), onto the J12 jumpers per J12_PINOUT.html.
  7. IMU + power leads up through the plate's front opening.
  8. Box up under the plate: the 4 keys drop into the 4 tab slots, 4x M3 from above.
  9. Battery under the floor, straps through the 4 ears, plug into the XT60.

MEASURE FIRST (change the number in src/box_rev1.py, run it again)
  - Kit outline: KIT_L x KIT_W = 71.5 x 51.0
  - Kit mounting hole: HOLE_FROM_J5_EDGE = 25.8 (from the USB-A side edge) - this one matters;
    HOLE_FROM_USB_EDGE = 19.2 only slides the board forward/back, the box has room for that
  - Bluefruit: 22.86 wide (header edge) x 25.4, 1.6 thick -> BLE_W, BLE_H, BLE_T
  - Tab slot positions on your plate: SLOT_X = 70.1, SLOT_Y = 17.0
  - Landing gear: the battery hangs 74.5 mm under the plate - the legs must be taller

REV 2
  Real mount for the power / kill board (relay, BJT, XT60, wiring) from Ethan's pictures.

REGENERATE
  pip install cadquery==2.8.0 vtk
  python src/box_rev1.py out          (STEP/STL + fit check)
  python src/render_rev1.py out/renders
