SENSOR DECK v3 (Nov 18, 2026) - IMU + Bluefruit + hub strip, middle of the bottom plate
=========================================================================================
Replaces the IMU slot adapter v2. Same keys into the 3 center slots, same 2 screws in the middle
slot, so the IMU still sits square in the exact middle of the frame. The Bluefruit sits just to
its LEFT, out of the box (better signal), antenna end pointing outward.

THE HUB STRIP (renders/0_hub_wiring_diagram.png)
  A 9 x 4 hole piece of your proto board (23 x 10 mm) lies ON TOP of the two pin rows:
  the IMU's pins and the Bluefruit's pins both come up through it from below. The shared lines
  are joined on the strip, and a 7-pin header on the strip is the ONE cable to J12.
  Rows, from the IMU outward:  1 = IMU pins   2 = cable header   3 = free   4 = Bluefruit pins
  Joins:  8 solder bridges (6 IMU pin -> cable pin, Bluefruit GND -> row 3 -> cable GND)
          5 short insulated wire links (3V3, MOSI, SCK, MISO, CS BLE) - see the diagram
  Not connected: Bluefruit RST, DFU, IRQ (the firmware polls), the IMU's other row.

  CABLE (7 plug-in jumpers, no cutting):   GND -> J12 6    3V3 -> J12 1    MOSI -> J12 37
                                           SCK -> J12 23   MISO -> J12 7   CS IMU -> J12 16
                                           CS BLE -> J12 13
  It runs forward and down through the bottom plate's big FRONT center opening into the box.

BUILD ORDER
  1. Print print/sensor_deck_v3_normal.stl (already flipped: flat top on the bed, keys up,
     no supports, 0.2 mm, 3 walls, 40%). Loose in the slots -> snug.stl, won't go in -> loose.stl.
  2. IMU on the deck with 4x M2.5 x 4-5 self-tapping. Its X ARROW points the same way as the
     deck's FRONT arrow -> its GND ... CS row is on the left, next to the Bluefruit spot.
     (In your Nov 18 photo the X arrow pointed LEFT: turn the board 90 deg clockwise.)
  3. Cut a 9 x 4 hole strip from the proto board.
  4. Push a 7-pin straight header into row 2 FROM UNDERNEATH (black plastic under the strip,
     long pins sticking up). Slide the plastic to the very bottom of the pins.
  5. Lower the strip onto the IMU's GND...CS pins (row 1). Set the Bluefruit on the deck,
     module side up, header toward the IMU, so its 9 pins come up through row 4.
     Screw the Bluefruit down: 2x M2 x 5 self-tapping in its 2 far-end holes.
  6. Solder every pin on top. Then the 8 bridges and 5 wire links from the diagram. Trim.
  7. Meter check: each cable pin beeps to its IMU pin and its Bluefruit pin (diagram),
     and no two cable pins beep to each other.
  8. Deck on the plate: keys into the 3 center slots, 2x M2.5 x 8 + nuts through the middle slot.

SIZE   deck 44 x 57.6 x 5.4 mm, about 8 g solid (less at 40% infill).
       Strip top 9.7 mm above the plate; with the cable plugged in about 24 mm (plates are 36.8 apart).

MEASURE FIRST (src/sensor_deck_v3.py)
  ROW_IMU = 11.43   the IMU's pin rows, from its center (0.05" in from the edge)
  BLE_W x BLE_L = 22.86 x 25.4, pin row 1.27 in from the header edge, holes 17.78 apart
  If the left ESC cable on the plate runs under the Bluefruit spot, move it outward first.
