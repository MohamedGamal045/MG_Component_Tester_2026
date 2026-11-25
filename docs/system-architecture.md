# System Architecture

## Controller

The project uses an Arduino Nano compatible board based on ATmega328P at 16 MHz and 5 V. Firmware `MG_V1.1` performs measurements in software and drives a graphical OLED interface.

## Measurement Flow

1. User inserts an unpowered component into TP1–TP3.
2. The firmware waits for the TEST button on A3.
3. Blue RGB state indicates testing.
4. The probe network drives combinations of TP1, TP2 and TP3 through 680 Ω and 470 kΩ paths.
5. The ADC and internal bandgap/auto-reference logic calculate candidate component values.
6. The component engine identifies and renders the most likely component result.
7. Green indicates a recognized/result state; red indicates the firmware fault/no-part heuristic.

## Firmware Architecture

- `setup()` initializes the button, RGB, OLED, ADC and pull-up behavior.
- `loop()` waits for a button sequence and starts one measurement cycle.
- The measurement engine applies probe combinations and stores resistor, diode, transistor, FET, capacitor, inductance and voltage-loss results.
- `mgResultScreen()` adds OLED symbols, values and pin mappings before setting the RGB result state.

## Evidence Boundaries

The source and PCB screenshots support the architecture. Fabricated PCB measurements, calibrated accuracy and production-test records are not included.
