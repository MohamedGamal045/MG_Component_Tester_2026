# Hardware Documentation

## Controller and Display

| Component | Implementation |
|---|---|
| MCU | ATmega328P, 16 MHz, 5 V |
| OLED | SSD1306, 128×64, I2C |
| OLED address | `0x3C` |
| SDA | A4 |
| SCL | A5 |

## Inputs and Outputs

| Interface | Pin | Behavior |
|---|---|---|
| TP1 | A0 / PC0 | Probe input |
| TP2 | A1 / PC1 | Probe input |
| TP3 | A2 / PC2 | Probe input |
| TEST | A3 / digital 17 | Active-low button model |
| RGB blue | D4 | Testing |
| RGB green | D5 | Valid measured result |
| RGB red | D6 | Fault/check |

## Probe Network

The source models nominal low resistance as `6800` in 0.1 Ω-scaled units and nominal high resistance as `47000` scaled for calculation. PCB resistance tolerance and trace routing are unverified.

## PCB Resources

- [Proteus project](../PCB/MG_Component_Tester_PCB_V1.2.pdsprj)
- [Top preview](../media/pcb/PCB_Top_2026-11-26%20005946.png)
- [Bottom preview](../media/pcb/PCB_Buttom_2026-11-26%20005946.png)
- [Schematic preview](../media/pcb/Schematic_Screenshot%202026-11-26%20005849.png)

No DRC report, Gerber set, BOM or assembly drawing is included in the workspace.
