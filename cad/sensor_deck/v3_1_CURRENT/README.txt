SENSOR DECK v3.1 (Nov 22, 2026) - IMU + Bluefruit + hub strip, middle of the bottom plate
=========================================================================================
Replaces sensor deck v3 (the blue one in your Nov 22 photos).

WHY v3 DIDN'T WORK
  v3 was drawn when we still thought the red arms were the front, so its keys point the deck's
  arrow at the TRUE RIGHT. You turned the IMU so its X arrow points to the true front, which is
  right, but then its GND ... CS row faces the true LEFT while the Bluefruit sat in FRONT of it with
  its pins facing the IMU's Qwiic side. The two pin rows were at 90 degrees, far apart.

WHAT v3.1 CHANGES
  - IMU, Bluefruit and hub strip turned 90 degrees on the deck. The keys, slots and screw
    positions are the same as v3, so it drops into the same 3 slots with the same 2 screws.
  - IMU in the exact center, X arrow -> TRUE FRONT (engraved arrow on the deck, right of the IMU).
    Its GND ... CS row faces the TRUE LEFT.
  - Bluefruit on the TRUE LEFT (white arms side), header facing the IMU: its 9 pins are 7.6 mm
    from the IMU's 6 pins, parallel, so ONE 9 x 4 hole strip sits on both rows (same as the v3 plan).
    The antenna end sits over the plate's left edge opening (no copper under the antenna).
  - Both deck screws are now sunk flush (counterbores), because the left one is under the hub
    strip and the Bluefruit. The deck is 1 mm thicker (5 mm) so there is still 2.3 mm under the heads.
    The same M2.5 x 8 socket screws + nuts still fit (4.1 mm below the plate, nut is 2.0).
  - The pocket under the Bluefruit's header solder joints is now blind (v3 cut it through).

HUB STRIP WIRING: same as v3, renders/0_hub_wiring_diagram.png ("FRONT" on it is now the true front)
  Rows from the IMU out:  1 = IMU pins   2 = 7-pin cable header   3 = free   4 = Bluefruit pins
  8 solder bridges + 5 short insulated wire links (3V3, MOSI, SCK, MISO, CS BLE). Everything
  soldered on TOP. Bluefruit RST, DFU, IRQ and the IMU's other row (SDOX ... INT2) not connected.
  CABLE (7 plug-in jumpers):  GND -> J12 6   3V3 -> J12 1   MOSI -> J12 37   SCK -> J12 23
                              MISO -> J12 7  CS IMU -> J12 16   CS BLE -> J12 13
  It goes down through the plate's front or back edge opening (into the box's wire bay),
  whichever reaches J12 better - both are about 35 mm from the strip.

PRINT (Creality K2 Pro, standard PLA)
  print/sensor_deck_v3_1_normal.stl - already flipped: flat top on the bed, keys up, NO supports.
  0.2 mm layers, 4 walls, 40% infill, nozzle 215 C, bed 55 C, elephant-foot compensation 0.15 mm.
  About 9 g, roughly half an hour. The screw counterbores and the Bluefruit pocket face the bed;
  their ceilings are short bridges (4 mm max), which print fine.
  Keys loose in the slots -> snug.stl. Keys won't go in -> loose.stl (same as v3).

BUILD ORDER (screws first - they're under the boards)
  1. Deck on the plate, ENGRAVED ARROW TO THE TRUE FRONT (the keys also fit turned 180 degrees,
     so check: the Bluefruit wing must be on the white-arms side). 2x M2.5 x 8 down into the
     counterbores, nuts under the plate.
  2. Straighten the IMU's GND ... CS pins (in your photo they're bent outward). They have to stand
     straight up to go through the strip.
  3. IMU on the deck, X arrow along the engraved arrow, 4x M2.5 self-tapping (same as now).
     Its GND ... CS row ends up next to the Bluefruit spot.
  4. Cut a 9 x 4 hole strip from your proto board. Push the 7-pin straight header into row 2 FROM
     UNDERNEATH (plastic under the strip, long pins up), plastic slid to the bottom of the pins.
  5. Bluefruit on the wing, module side up, header toward the IMU, 2x M2 x 5 self-tapping at its far
     end. Lower the strip onto both pin rows (IMU pins through row 1, Bluefruit pins through row 4).
  6. Solder every pin on top, then the 8 bridges and 5 links from the diagram. Trim.
  7. Meter check: each cable pin beeps to its IMU pin and its Bluefruit pin; no two cable pins beep
     to each other.
  8. Before tightening anything for good: look at where the LEFT ESC leads run on the plate. The
     wing covers x = -44.6 ... -10 mm, y = -11.6 ... +14.1 mm from the plate center. If a lead runs
     there, move it outward first.

FIT CHECK (fit_check_v3_1.txt)
  0 overlap: keys vs slots; deck vs IMU, Bluefruit, hub strip; screw heads vs deck, IMU, Bluefruit,
  strip, cable header, pins. IMU X arrow -> +Y (true front). Bluefruit wing corner 14.4 mm inside
  the line between the front-left arm's 2 plate screws, 15.5 mm for the back-left arm.
  Plugged cable top about 25 mm above the plate (plates are 36.8 apart).

FILES
  print/   sensor_deck_v3_1_snug / normal / loose .stl
  step/    sensor_deck_v3_1.step (deck), sensor_deck_v3_1_assembly.step (plate, deck, IMU, Bluefruit,
           strip, cable header, pins, links, screw heads)
  renders/ 0 hub wiring diagram, 1 top view (true front up), 2 assembled, 3 hub close-up,
           4 deck alone, 5 print orientation
  src/     sensor_deck_v3_1.py (all numbers at the top; run: python sensor_deck_v3_1.py OUT_DIR),
           render_v3_1.py, hub_diagram.py, f450_bottom_plate.json
