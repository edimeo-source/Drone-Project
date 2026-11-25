F450 REPLACEMENT LANDING LEG v1 - Ethan DiMeo - Nov 25, 2026
================================================================

One of the 4 stock tall legs broke on the tether. This is a printed copy: it hangs on the
same 2 arm screws, stands the plate 150 mm off the ground like the stock legs, and weighs
about the same, so the drone's balance doesn't change.

FILES
  print/f450_leg_print.stl    slice this one (already in print position: bracket face down)
  print/f450_leg_as_mounted.stl  the same leg in drone position (for CAD / checking)
  step/f450_leg.step          edit in Fusion 360
  leg_drawing.pdf / .png      dimensioned drawing: hold it against one of your good legs
  renders/                    pictures (alone, underside, on the drone, on the bed)
  src/f450_leg.py             the model; every size is a parameter at the top
  src/fit_check.py            fit against collar v3.1, tray v4.1 and the plate
  src/print_check.py          overhang / watertight / weight check of the print STL
  src/render_leg.py, src/drawing.py   pictures and drawing
  fit_check.txt, print_check.txt      results

WHERE THE NUMBERS COME FROM
  Bracket 23.2 x 28.5 x 7.3, nose 10.6 flat and 18.5 in from the screw line,
  chamfers 10.5 in, leg 14.9 wide, rib from the nose ... your calipers + photos (Nov 20),
     the same numbers collar v3.1 was built on (and it fits the real legs).
  Screws 20.5 apart ............................. the plate's arm screws (collar v3.1)
  150 mm plate to ground, ~200 mm long .......... F450 tall-leg listings
  Leaves the bracket, drops to 20 mm, then leans out 0.95 mm per mm down
                                                 ... the photo trace used for collar v3.1
  20.8 g ........................................ frame listed 395 g, measured 312 g without
                                                  the 4 legs: (395 - 312) / 4 = 20.75 g
                                                  (tall-leg listings say 18 to 22.5 g)

  NOT measured on your leg yet (my best numbers): the 150 mm height, how far out the foot is,
  the leg's depth from the side (15 mm here) and the weight. See CHECK BEFORE PRINTING.

CHECK BEFORE PRINTING (with one of your 3 good legs, 2 minutes)
  1. Plate underside to the ground, drone standing level ................ model: 150 mm
  2. Screw line (the 2 screws) straight out to the tip of the foot ...... model: 149 mm
  3. Leg thickness seen from the side, halfway down ..................... model: 15 mm
  4. Weight on the kitchen scale ......................................... model: 20.8 g
  If any is off by more than 3 mm or 1 g, send the numbers (or a side photo next to a
  ruler) and the model gets re-made: they are parameters (HEIGHT, SLOPE, DEPTH, TARGET_G),
  and the wall thickness re-solves itself to hit the weight.

WEIGHT AND BALANCE
  A leg that is off by dm grams, about 190 mm from the center, moves the center of
  gravity by about  dx = dm x 190 / 1550  mm  (1550 g = drone + battery).
  2 g off -> 0.25 mm. The PIDs' integral term takes that out; it does not matter.

PRINT (Creality K2 Pro, standard PLA)
  Orientation: as in f450_leg_print.stl - bracket face (the face that touches the plate) DOWN.
               The leg leans up and out at 46 deg from the bed. Do not lay it on its side.
  Supports: OFF. Every overhang is 44 deg or steeper (print_check.txt: 0.1 mm2 flatter).
  Brim: 5 mm (the bracket is the only part on the bed and the leg leans).
  Infill: 100 %. The I-beam walls are 1.30 / 1.46 mm, so they print solid anyway, and
          100 % makes the weight come out right (about 20.8 g).
  Layers 0.20 mm. Nozzle 220 C, bed 55 C, fan 80 % from layer 3 (a little hotter and less fan
  than the box, for stronger layers in the leg).
  Outer walls 100 mm/s. PLA used: about 21 g plus the brim.

FIT
  Reuse the 2 screws from the broken leg. Holes are 3.2 mm (M2.5 or M3 screws).
  If an M3 screw is tight, run a 3.2-3.4 mm drill through by hand.
  The new part is inside the shape the old leg took up, except a small 45 deg wedge off the
  bracket's outer end (it holds the leg's top up while printing). It stays below the plate,
  and the fit check shows 0 overlap with collar v3.1, tray v4.1 and the plate at all 4 corners.
  If an ESC or a wire sits under the arm right at the plate edge, check that wedge clears it.

STRENGTH
  The stock legs are thin ABS and break easily (the frame reviews say the same).
  This copy:
    - is solid where the leg leaves the bracket (where they snap);
    - has a rib from the nose like the stock leg;
    - has an I-beam the rest of the way.
  PLA is stiffer but more brittle than ABS, and printed upright its layers run across the leg,
  so a hard sideways landing can still snap it at the top.
  Print 2 and keep one as a spare. PETG is tougher if you have it (set PLA = 1.27 in
  f450_leg.py for PETG's density and re-run it to keep the weight right).
