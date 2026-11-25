# Testing and Validation

## Completed Evidence

- Source-level review of the ATmega328P firmware.
- Source-level verification of OLED, RGB and TEST mappings.
- PCB project review and top/bottom/schematic image capture.
- Prototype assembly and operating-footage capture.

## Required Before Release

1. Verify in Arduino IDE for the exact Nano processor option.
2. Confirm flash and RAM remain within the target board limits.
3. Upload and confirm the OLED address `0x3C`.
4. Confirm the RGB module is common-anode and D4/D5/D6 wiring matches the PCB.
5. Measure all three probe networks with a calibrated meter.
6. Test known resistors, capacitors, diodes, LEDs, BJTs, MOSFETs, inductors and power-off cells.
7. Compare readings against calibrated instruments.
8. Record repeatability and temperature behavior.
9. Test red fault behavior with empty sockets and live-voltage probes.
10. Export Gerber, drill, BOM and assembly data before fabrication handoff.
