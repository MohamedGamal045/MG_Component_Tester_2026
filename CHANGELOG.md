# Changelog

## MG_V1.1 / PCB V1.2 workspace

### Added
- OLED 128×64 I2C interface at SSD1306 address `0x3C`.
- Graphical component symbols and pin-out/result screens.
- Common-anode RGB status behavior: blue testing, green measured, red fault/check.
- MG Engineer branding and custom boot bitmap.
- TP2 capacitance offset correction and OLED top-offset compensation.
- English project and function-level firmware documentation.

### Changed
- Default build is OLED-only; unused HardwareSerial behavior is disabled.
- UART transmit mask is cleared in the no-UART build to protect A3/A4/A5.
- OLED reset is disabled for a four-wire I2C module (`OLED_RESET = -1`).

### Known validation
- Arduino AVR compile and upload were not available in the audited workspace.
- PCB resistor values, RGB wiring, display module, and measurement accuracy require bench verification.
