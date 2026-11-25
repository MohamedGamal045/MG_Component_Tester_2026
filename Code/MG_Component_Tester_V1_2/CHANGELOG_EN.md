# MG Component Tester — Firmware Changes

## Build-size recovery
- Removed unused Arduino HardwareSerial initialization and character-by-character mirroring from the default OLED-only build.
- Kept existing resistor, capacitor, diode, transistor, MOSFET, inductance, ESR and voltage-loss measurement procedures unchanged.

## Electrical fixes
- Set `TXD_MSK` to 0 in the no-UART build. This prevents unrelated A3/A4/A5 pins from being driven by the ADC port-direction assignments, preserving the button and I2C bus.
- Disabled the nonexistent external OLED reset line on a four-wire I2C module (`OLED_RESET -1`).
- Added explicit failure handling if the SSD1306 device cannot be initialized.

## UI and maintainability
- Preserved three-page startup (project, supervisors, 11 members), graphical results, RGB statuses, and top OLED offset.
- Added concise English function-level documentation and project metadata.
- Replaced decorative original header with a new accurate derivative-project attribution.

## Validation limitation
- Source-level review completed. Arduino AVR toolchain is unavailable here, so compile/link size, hardware accuracy and upload are **not confirmed**. Run Arduino IDE Verify with the same board package to measure the actual bytes used.
