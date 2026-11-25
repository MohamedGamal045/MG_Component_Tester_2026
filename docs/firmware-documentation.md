# Firmware Documentation

## Source

[MG_Component_Tester_V1_2.ino](../Code/MG_Component_Tester_V1_2/MG_Component_Tester_V1_2.ino)

## Build Configuration

```text
Board: Arduino Nano
Processor: ATmega328P
Clock: 16 MHz
Supply: 5 V
Display: SSD1306 I2C
I2C address: 0x3C
UART: disabled
Self-test: disabled
Auto-calibration: disabled
Internal auto-reference: enabled
```

## Measurement Categories

The code declares result categories for resistors, inductors, capacitors, diodes, LEDs, BJTs, MOSFET/JFETs, TRIACs and thyristors. Inductors are rendered when one resistor-class result contains inductive information; LEDs are treated as a diode with forward voltage at or above 1350 mV.

## Safety Logic

The result screen marks the following states faulty:

- No component or cell state
- A one-resistor result with zero resistance and inductance
- Capacitor loss above `150` in the source scaling or ESR above `50 Ω`
- BJT hFE below `5` or base-emitter voltage outside `100–2500 mV`

These are software heuristics. They do not prove component safety or suitability.
