# Communication and Interfaces

## I2C Display

| Parameter | Value |
|---|---|
| Bus | I2C via Arduino Wire |
| Address | `0x3C` |
| SDA | A4 |
| SCL | A5 |
| Device | SSD1306, 128×64 |

## Analog and GPIO

The firmware uses PC0–PC2 for TP1–TP3, A3 as a TEST button, D4–D6 for RGB status, and D8–D13 for probe-network drive combinations in the core measurement engine. The source also references A4/A5 for OLED, so these pins must remain reserved rather than used for probe reference or battery ADC functions in the active OLED build.

## Serial

Hardware/UART support is disabled by default. No tested packet format or external radio/host protocol is implemented by the current source.
