/*
 * ============================================================================
 * PROJECT: MG_V1.2 - MG Engineer Component Tester & Measurement System
 * AUTHOR:  Mohamed Gamal (MG Engineer)
 * WEB:     https://sites.google.com/view/mgengineer
 * GITHUB:  https://github.com/MohamedGamal045
 * BOARD:   Arduino Nano (ATmega328P, 5 V, 16 MHz)
 * DISPLAY: SSD1306 OLED, 128 x 64, I2C address 0x3C
 *
 * OWNERSHIP & DEVELOPMENT NOTICE:
 * Developed, refactored, and engineered by Mohamed Gamal (MG Engineer).
 * While this firmware references the core measurement engine from
 * Transistor Tester for Arduino v1.08a (originally based on work by
 * Karl-Heinz Kubbeler, v1.08k), virtually the entire codebase has been
 * extensively modified, upgraded with new hardware/measurement capabilities,
 * and rebuilt with a completely custom Graphical User Interface (GUI).
 *
 * KEY FEATURES & CUSTOMIZATIONS (MG_V1.1):
 * - Custom MG boot logo bitmap & MG Engineer brand splash sequence.
 * - Full graphical component-result GUI with schematic symbols & pinouts.
 * - True anti-parallel TRIAC schematic symbol & MT1/G/MT2 pinout.
 * - Dedicated Inductor detection UI with 4-loop coil symbol, L (H) & R (Ohm).
 * - Dedicated LED detection UI with outward-arrow symbol & Vf readout.
 * - RGB status indicator (Common Anode: D6=Red Fault, D5=Green OK, D4=Blue Busy).
 * - OLED vertical top-offset compensation (MG_OLED_TOP_OFFSET).
 * - Improved ADC rounding (/1024), bandgap settling, bidirectional R averaging,
 *   TP2 capacitance offset correction, MOSFET Cg measurement, and hFE/Vt precision.
 *
 * IMPORTANT: Measure only unpowered, discharged components.
 * ============================================================================
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdint.h>
#include <avr/power.h>

//#define LCD1602
//#define LCD_I2C
//#define NOK5110
//#define OLED096
#define OLED_I2C

#ifdef LCD_I2C
  #ifndef LCD1602
    #define LCD1602
  #endif
#endif

#ifdef OLED_I2C
  #ifndef OLED096
    #define OLED096
  #endif
#endif

#ifdef LCD1602
  #ifdef LCD_I2C
    #include <Wire.h> 
    #include <LiquidCrystal_I2C.h>
  #else
    #include <LiquidCrystal.h>
  #endif
#endif

#ifdef NOK5110
  #include <SPI.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_PCD8544.h>
#endif

#ifdef OLED096
  #ifndef OLED_I2C
    #include <SPI.h>
  #endif
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif


// SECTION: Compile-time settings for the measuring engine and OLED build.
// ******** config options for your Semiconductor tester

#define MCU atmega328p
#define F_CPU 16000000UL

// Select your language:
#define LANG_ENGLISH

// The LCD_CYRILLIC option is necessary, if you have a display with cyrillic characterset.
#define LCD_CYRILLIC

// The WITH_SELFTEST option enables selftest function (only for mega168 or mega328).
//#define WITH_SELFTEST

// AUTO_CAL will enable the autocalibration of zero offset of capacity measurement
//#define AUTO_CAL

// FREQUENCY_50HZ enables a 50 Hz frequency generator at the end of selftests.
//#define FREQUENCY_50HZ

// The WITH_AUTO_REF option enables reading of internal REF-voltage to get factors for the Capacity measuring.
#define WITH_AUTO_REF
// REF_C_KORR corrects the reference Voltage for capacity measurement (<40uF) and has mV units.
#define REF_C_KORR 12
// REF_L_KORR corrects the reference Voltage for inductance measurement and has mV units.
#define REF_L_KORR 40
// C_H_KORR defines a correction of 0.1% units for big capacitor measurement.
#define C_H_KORR 0

// The WITH_UART option enables the software UART (disabled by default to save Flash & protect A3-A5).
//#define WITH_UART

// The CAP_EMPTY_LEVEL defines the empty voltage level for capacitors in mV.
#define CAP_EMPTY_LEVEL 4

// The AUTOSCALE_ADC option enables the autoscale ADC (ADC use VCC and Bandgap Ref).
#define AUTOSCALE_ADC
#define REF_R_KORR 3

// The ESR_ZERO value defines the zero value of ESR measurement (units = 0.01 Ohm).
#define ESR_ZERO 20

// NO_AREF_CAP tells your Software, that you have no Capacitor installed at pin AREF (21).
#define NO_AREF_CAP

// The PULLUP_DISABLE option disables the pull-up Resistors of IO-Ports.
#define PULLUP_DISABLE

// The ANZ_MESS option specifies how often an ADC value is read and accumulated.
#define ANZ_MESS 64

// Battery check threshold
#define BAT_POOR 6400

// ******** end of selectable options

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// SECTION: Register mapping, analog inputs and 680-ohm/470-kohm probe network.
#ifndef ADC_PORT

#define ADC_PORT PORTC
#define ADC_DDR DDRC
#define ADC_PIN PINC
#define TP1 0
#define TP2 1
#define TP3 2
#define TPext 3
// Port pin for 2.5V precision reference used for VCC check (optional)
#define TPREF 4
// Port pin for Battery voltage measuring
#define TPBAT 5

/*
  exact values of used resistors (Ohm).
  The standard value for R_L is 680 Ohm, for R_H 470kOhm.
*/
#define R_L_VAL 6800          // standard value 680 Ohm, multiplied by 10 for 0.1 Ohm resolution
#define R_H_VAL 47000         // standard value 470000 Ohm, multiplied by 10, divided by 100 

#define R_DDR DDRB
#define R_PORT PORTB

#define ON_DDR DDRD
#define ON_PORT PORTD
#define ON_PIN_REG PIND
#define ON_PIN 18               // Pin, must be switched to high to switch power on

#ifdef STRIP_GRID_BOARD
  #define RST_PIN 0             // Pin, is switched to low, if push button is pressed
#else
  #define RST_PIN 17            // Pin, is switched to low, if push button is pressed
#endif

// Port(s) / Pins for LCD
#ifdef STRIP_GRID_BOARD
  #define HW_LCD_EN_PORT         PORTD
  #define HW_LCD_EN_PIN          5
  #define HW_LCD_RS_PORT         PORTD
  #define HW_LCD_RS_PIN          7
  #define HW_LCD_B4_PORT         PORTD
  #define HW_LCD_B4_PIN          4
  #define HW_LCD_B5_PORT         PORTD
  #define HW_LCD_B5_PIN          3
  #define HW_LCD_B6_PORT         PORTD
  #define HW_LCD_B6_PIN          2
  #define HW_LCD_B7_PORT         PORTD
  #define HW_LCD_B7_PIN          1
#else
  #define HW_LCD_EN_PORT         PORTD
  #define HW_LCD_EN_PIN          6
  #define HW_LCD_RS_PORT         PORTD
  #define HW_LCD_RS_PIN          7
  #define HW_LCD_B4_PORT         PORTD
  #define HW_LCD_B4_PIN          5
  #define HW_LCD_B5_PORT         PORTD
  #define HW_LCD_B5_PIN          4
  #define HW_LCD_B6_PORT         PORTD
  #define HW_LCD_B6_PIN          3
  #define HW_LCD_B7_PORT         PORTD
  #define HW_LCD_B7_PIN          2
#endif

// U_VCC defines the VCC Voltage of the ATmega in mV units
#define U_VCC 5000

// U_SCALE can be set to 4 for better resolution of ReadADC function for resistor measurement
#define U_SCALE 4

// R_ANZ_MESS can be set to a higher number of measurements (up to 200) for resistor measurement
#define R_ANZ_MESS 200

#if R_ANZ_MESS < ANZ_MESS
  #undef R_ANZ_MESS
  #define R_ANZ_MESS ANZ_MESS
#endif
#if U_SCALE < 0
  #undef U_SCALE
  #define U_SCALE 1
#endif
#if U_SCALE > 4
  #undef U_SCALE
  #define U_SCALE 4
#endif
#ifndef REF_L_KORR
  #define REF_L_KORR 50
#endif

// External data location: EEPROM or Flash
#ifdef USE_EEPROM
  #define MEM_TEXT EEMEM

  #if E2END > 0X1FF
    #define MEM2_TEXT EEMEM
    #define MEM2_read_byte(a)  eeprom_read_byte(a)
    #define MEM2_read_word(a)  eeprom_read_word(a)
    #define lcd_fix2_string(a)  lcd_fix_string(a)
  #else
    #define MEM2_TEXT PROGMEM
    #define MEM2_read_byte(a)  pgm_read_byte(a)
    #define MEM2_read_word(a)  pgm_read_word(a)
    #define lcd_fix2_string(a)  lcd_pgm_string(a)
    #define use_lcd_pgm
  #endif

  #define MEM_read_word(a)  eeprom_read_word(a)
  #define MEM_read_byte(a)  eeprom_read_byte(a)

#else
  #define MEM_TEXT PROGMEM
  #define MEM2_TEXT PROGMEM
  #define MEM_read_word(a)  pgm_read_word(a)
  #define MEM_read_byte(a)  pgm_read_byte(a)
  #define MEM2_read_byte(a)  pgm_read_byte(a)
  #define MEM2_read_word(a)  pgm_read_word(a)
  #define lcd_fix2_string(a)  lcd_pgm_string(a)
  #define use_lcd_pgm
#endif

// RH_OFFSET : systematic offset of resistor measurement with RH (470k) 
#define RH_OFFSET 3500 

// TP2_CAP_OFFSET is an additional offset for TP2 capacity measurements in pF units
#define TP2_CAP_OFFSET 2

// CABLE_CAP defines the capacity (pF) of 12cm cable with clip at the terminal pins
#define CABLE_CAP 3

#define PROCESSOR_TYP 328

#if FLASHEND > 0X1FFF
  #define ACALL call
#else
  #define ACALL rcall
#endif

#if PROCESSOR_TYP == 168
  #define MCU_STATUS_REG MCUCR
  #define ADC_COMP_CONTROL ADCSRB
  #define TI1_INT_FLAGS TIFR1
  #define DEFAULT_BAND_GAP 1070
  #define DEFAULT_RH_FAKT  884
  #define LONG_HFE
  #define COMMON_COLLECTOR
  #define PIN_RM 190
  #define PIN_RP 220
  #define CC0 36
  #define COMP_SLEW1 4000
  #define COMP_SLEW2 220
  #define C_NULL CC0+CABLE_CAP+(COMP_SLEW1 / (CC0 + CABLE_CAP + COMP_SLEW2))
  #define MUX_INT_REF 0x0e
#elif PROCESSOR_TYP == 328
  #define MCU_STATUS_REG MCUCR
  #define ADC_COMP_CONTROL ADCSRB
  #define TI1_INT_FLAGS TIFR1
  #define DEFAULT_BAND_GAP 1070
  #define DEFAULT_RH_FAKT  884
  #define LONG_HFE
  #define COMMON_COLLECTOR
  #define PIN_RM 200
  #define PIN_RP 220
  #define CC0 36
  #define COMP_SLEW1 4000
  #define COMP_SLEW2 180
  #define C_NULL CC0+CABLE_CAP+(COMP_SLEW1 / (CC0 + CABLE_CAP + COMP_SLEW2))
  #define MUX_INT_REF 0x0e
#elif PROCESSOR_TYP == 1280
  #define MCU_STATUS_REG MCUCR
  #define ADC_COMP_CONTROL ADCSRB
  #define TI1_INT_FLAGS TIFR1
  #define DEFAULT_BAND_GAP 1070
  #define DEFAULT_RH_FAKT  884
  #define LONG_HFE
  #define COMMON_COLLECTOR
  #define PIN_RM 200
  #define PIN_RP 220
  #define CC0 36
  #define COMP_SLEW1 4000
  #define COMP_SLEW2 180
  #define C_NULL CC0+CABLE_CAP+(COMP_SLEW1 / (CC0 + CABLE_CAP + COMP_SLEW2))
  #define MUX_INT_REF 0x1e
#else
  #define MCU_STATUS_REG MCUCSR
  #define ADC_COMP_CONTROL SFIOR
  #define TI1_INT_FLAGS TIFR
  #define DEFAULT_BAND_GAP 1298
  #define DEFAULT_RH_FAKT  740
  #define LONG_HFE
  #define COMMON_COLLECTOR
  #define PIN_RM 196
  #define PIN_RP 240
  #define CC0 27
  #define COMP_SLEW1 0
  #define COMP_SLEW2 33
  #define C_NULL CC0+CABLE_CAP+(COMP_SLEW1 / (CC0 + CABLE_CAP + COMP_SLEW2))
  #define MUX_INT_REF 0x0e
  #ifndef INHIBIT_SLEEP_MODE
    #define INHIBIT_SLEEP_MODE
  #endif
#endif

#if PROCESSOR_TYP == 8
  #ifdef AUTO_CAL
    #define ADC_internal_reference (2560 + (int8_t)eeprom_read_byte((uint8_t *)&RefDiff))
  #else
    #define ADC_internal_reference (2560 + REF_R_KORR)
  #endif
#else
  #ifdef AUTO_CAL
    #define ADC_internal_reference (ref_mv + (int8_t)eeprom_read_byte((uint8_t *)&RefDiff))
  #else
    #define ADC_internal_reference (ref_mv + REF_R_KORR)
  #endif
#endif

#ifndef REF_R_KORR
  #define REF_R_KORR 0
#endif
#ifndef REF_C_KORR
  #define REF_C_KORR 0
#endif

#define LONG_WAIT_TIME 28000
#define SHORT_WAIT_TIME 5000

#ifdef POWER_OFF
  #if (POWER_OFF+0) > 2
    #define OFF_WAIT_TIME SHORT_WAIT_TIME
  #else
    #define OFF_WAIT_TIME LONG_WAIT_TIME
  #endif
#else
  #define OFF_WAIT_TIME  LONG_WAIT_TIME
#endif

//**********************************************************
#define F_ADC 125000
#if F_CPU/F_ADC == 2
  #define AUTO_CLOCK_DIV (1<<ADPS0) 
#elif F_CPU/F_ADC == 4
  #define AUTO_CLOCK_DIV (1<<ADPS1) 
#elif F_CPU/F_ADC == 8
  #define AUTO_CLOCK_DIV (1<<ADPS1) | (1<<ADPS0)
#elif F_CPU/F_ADC == 16
  #define AUTO_CLOCK_DIV (1<<ADPS2)
#elif F_CPU/F_ADC == 32
  #define AUTO_CLOCK_DIV (1<<ADPS2) | (1<<ADPS0)
#elif F_CPU/F_ADC == 64
  #define AUTO_CLOCK_DIV (1<<ADPS2) | (1<<ADPS1)
#elif F_CPU/F_ADC == 128
  #define AUTO_CLOCK_DIV (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)
#endif

#define F_ADC_F 500000
#if F_CPU/F_ADC_F == 2
  #define FAST_CLOCK_DIV (1<<ADPS0) 
#elif F_CPU/F_ADC_F == 4
  #define FAST_CLOCK_DIV (1<<ADPS1) 
#elif F_CPU/F_ADC_F == 8
  #define FAST_CLOCK_DIV (1<<ADPS1) | (1<<ADPS0)
#elif F_CPU/F_ADC_F == 16
  #define FAST_CLOCK_DIV (1<<ADPS2)
#elif F_CPU/F_ADC_F == 32
  #define FAST_CLOCK_DIV (1<<ADPS2) | (1<<ADPS0)
#elif F_CPU/F_ADC_F == 64
  #define FAST_CLOCK_DIV (1<<ADPS2) | (1<<ADPS1)
#elif F_CPU/F_ADC_F == 128
  #define FAST_CLOCK_DIV (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)
#endif

#ifndef PIN_RP
  #define PIN_RP  220
#endif
#ifndef PIN_RM
  #define PIN_RM  190
#endif

//**********************************************************
#define TxD 3	  // TxD-Pin of Software-UART; must be at Port C !
#ifdef WITH_UART
  #define TXD_MSK (1<<TxD)
#else
  // Protect PC3/A3 (TEST key) and PC4/PC5 (I2C); only PC0-PC2 are probes.
  #define TXD_MSK 0x00
#endif

#ifdef SWUART_INVERT
  #define TXD_VAL 0
#else
  #define TXD_VAL TXD_MSK
#endif

#ifdef INHIBIT_SLEEP_MODE
  #define wait_about5ms() wait5ms()
  #define wait_about10ms() wait10ms()
  #define wait_about20ms() wait20ms()
  #define wait_about30ms() wait30ms()
  #define wait_about50ms() wait50ms()
  #define wait_about100ms() wait100ms()
  #define wait_about200ms() wait200ms()
  #define wait_about300ms() wait300ms()
  #define wait_about400ms() wait400ms()
  #define wait_about500ms() wait500ms()
  #define wait_about1s() wait1s()
  #define wait_about2s() wait2s()
  #define wait_about3s() wait3s()
  #define wait_about4s() wait4s()
#else
  #define wait_about5ms() sleep_5ms(1)
  #define wait_about10ms() sleep_5ms(2)
  #define wait_about20ms() sleep_5ms(4)
  #define wait_about30ms() sleep_5ms(6)
  #define wait_about50ms() sleep_5ms(10)
  #define wait_about100ms() sleep_5ms(20)
  #define wait_about200ms() sleep_5ms(40)
  #define wait_about300ms() sleep_5ms(60)
  #define wait_about400ms() sleep_5ms(80)
  #define wait_about500ms() sleep_5ms(100)
  #define wait_about1s() sleep_5ms(200)
  #define wait_about2s() sleep_5ms(400)
  #define wait_about3s() sleep_5ms(600)
  #define wait_about4s() sleep_5ms(800)
#endif

#undef AUTO_RH
#ifdef WITH_AUTO_REF
  #define AUTO_RH
#else
  #ifdef AUTO_CAL
    #define AUTO_RH
  #endif
#endif

#undef CHECK_CALL
#ifdef WITH_SELFTEST
  #define CHECK_CALL
#endif

#ifdef AUTO_CAL
  #define CHECK_CALL
  #define RR680PL resis680pl
  #define RR680MI resis680mi
  #define RRpinPL pin_rpl
  #define RRpinMI pin_rmi
#else
  #define RR680PL (R_L_VAL + PIN_RP)
  #define RR680MI (R_L_VAL + PIN_RM)
  #define RRpinPL (PIN_RP)
  #define RRpinMI (PIN_RM)
#endif

#ifndef ESR_ZERO
  #define ESR_ZERO 20
#endif

#ifndef RESTART_DELAY_TICS
  #define RESTART_DELAY_TICS 16384
#endif

#if EBC_STYLE == 123
  #undef EBC_STYLE
#endif

#if defined(NOK5110) || defined(OLED096)
  #define LCD_CHAR_DIODE1 0x91
  #define LCD_CHAR_DIODE2 0x92
  #define LCD_CHAR_CAP    0x93
  #define LCD_CHAR_RESIS1 0x94
  #define LCD_CHAR_RESIS2 0x95
  #define LCD_CHAR_OMEGA  0x90
  #define LCD_CHAR_U      0xB5
#else
  #define LCD_CHAR_DIODE1  1
  #define LCD_CHAR_DIODE2  2
  #define LCD_CHAR_CAP 3
  #define LCD_CHAR_RESIS1 7
  #define LCD_CHAR_RESIS2 6

  #ifdef LCD_CYRILLIC
    #define LCD_CHAR_OMEGA  4
    #define LCD_CHAR_U  5
  #else
    #define LCD_CHAR_OMEGA  244
    #define LCD_CHAR_U  228
  #endif

  #ifdef LCD_DOGM
    #undef LCD_CHAR_OMEGA
    #define LCD_CHAR_OMEGA 0x1e
    #undef LCD_CHAR_U
    #define LCD_CHAR_U  5
  #endif

  #define LCD_CHAR_DEGREE 0xdf
#endif

#endif  // #ifndef ADC_PORT

#ifdef COMMON_COLLECTOR
  #if FLASHEND > 0x3fff
    #define COMMON_EMITTER
  #endif
#else
  #define COMMON_EMITTER
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

#define MAIN_C

#if defined (MAIN_C)
  #define COMMON

// Reference resistance: 680 ohms                300   325   350   375   400   425   450   475   500   525   550   575   600   625   650   675   700   725   750   775   800   825   850   875   900   925   950   975  1000  1025  1050  1075  1100  1125  1150  1175  1200  1225  1250  1275  1300  1325  1350  1375  1400  mV
const uint16_t RLtab[] MEM_TEXT = {22447,20665,19138,17815,16657,15635,14727,13914,13182,12520,11918,11369,10865,10401, 9973, 9577, 9209, 8866, 8546, 8247, 7966, 7702, 7454, 7220, 6999, 6789, 6591, 6403, 6224, 6054, 5892, 5738, 5590, 5449, 5314, 5185, 5061, 4942, 4828, 4718, 4613, 4511, 4413, 4319, 4228};

#if FLASHEND > 0x1fff
  const uint16_t LogTab[] PROGMEM = {0, 20, 41, 62, 83, 105, 128, 151, 174, 198, 223, 248, 274, 301, 329, 357, 386, 416, 446, 478, 511, 545, 580, 616, 654, 693, 734, 777, 821, 868, 916, 968, 1022, 1079, 1139, 1204, 1273, 1347, 1427, 1514, 1609, 1715, 1833, 1966, 2120, 2303, 2526 };
#endif

#ifdef AUTO_RH
  // resistor  470000 Ohm      1000 1050 1100 1150 1200 1250 1300 1350 1400  mV
  const uint16_t RHtab[] PROGMEM = { 954, 903, 856, 814, 775, 740, 707, 676, 648};
#endif

// Strings in PROGMEM
#if defined(LANG_ENGLISH)
  const unsigned char TestRunning[] MEM_TEXT = "testing...";
  const unsigned char BatWeak[] MEM_TEXT = "weak";
  const unsigned char BatEmpty[] MEM_TEXT = "empty!";
  const unsigned char TestFailed2[] MEM_TEXT = "damaged ";
  const unsigned char Component[] MEM_TEXT = "part";
  const unsigned char Triac[] MEM_TEXT = "Triac";
  const unsigned char Thyristor[] MEM_TEXT = "Thyristor";
  const unsigned char Unknown[] MEM_TEXT = " unknown";
  const unsigned char TestFailed1[] MEM_TEXT = "No, unknown, or";
  const unsigned char OrBroken[] MEM_TEXT = "or damaged ";
  const unsigned char TestTimedOut[] MEM_TEXT = "Timeout!";
  #define Cathode_char 'C'

  #ifdef WITH_SELFTEST
    const unsigned char SELFTEST[] MEM_TEXT = "Selftest mode..";
    const unsigned char RELPROBE[] MEM_TEXT = "isolate Probe!";
    const unsigned char ATE[] MEM_TEXT = "Test End";
  #endif
#endif

const unsigned char Bat_str[] MEM_TEXT = "Bat. ";
const unsigned char OK_str[] MEM_TEXT = "OK";
const unsigned char mosfet_str[] MEM_TEXT = "-MOS";
const unsigned char jfet_str[] MEM_TEXT = "JFET";
const unsigned char GateCap_str[] MEM_TEXT = "C=";
const unsigned char hfe_str[] MEM_TEXT ="B=";
const unsigned char NPN_str[] MEM_TEXT = "NPN ";
const unsigned char PNP_str[] MEM_TEXT = "PNP ";

#ifndef EBC_STYLE
  const unsigned char N123_str[] MEM_TEXT = " 123=";
#else
  #if EBC_STYLE == 321
    const unsigned char N321_str[] MEM_TEXT = " 321=";
  #endif
#endif

const unsigned char Uf_str[] MEM_TEXT = "Uf=";
const unsigned char vt_str[] MEM_TEXT = " Vt=";
const unsigned char Vgs_str[] MEM_TEXT = "@Vgs=";
const unsigned char CapZeich[] MEM_TEXT = {'-',LCD_CHAR_CAP,'-',0};
const unsigned char Cell_str[] MEM_TEXT = "Cell!";
const unsigned char VCC_str[] MEM_TEXT = "VCC=";

#if FLASHEND > 0x1fff
  const unsigned char ESR_str[] MEM_TEXT = " ESR=";
  const unsigned char VLOSS_str[] MEM_TEXT = " Vloss=";
  const unsigned char Lis_str[] MEM_TEXT = "L=";
  const unsigned char Ir_str[] MEM_TEXT = "  Ir=";
#endif

#ifdef WITH_VEXT
  const unsigned char Vext_str[] MEM_TEXT = "Vext=";
  #define LCD_CLEAR
#endif

const unsigned char VERSION_str[] MEM2_TEXT = "MG_V1.1";

const unsigned char AnKat[] MEM_TEXT = {'-', LCD_CHAR_DIODE1, '-',0};
const unsigned char KatAn[] MEM_TEXT = {'-', LCD_CHAR_DIODE2, '-',0};
const unsigned char Diodes[] MEM_TEXT = {'*',LCD_CHAR_DIODE1, ' ', ' ',0};
const unsigned char Resistor_str[] MEM_TEXT = {'-', LCD_CHAR_RESIS1, LCD_CHAR_RESIS2,'-',0};

#ifdef WITH_SELFTEST
  const unsigned char URefT[] MEM2_TEXT = "Ref=";
  const unsigned char RHfakt[] MEM2_TEXT = "RHf=";
  const unsigned char RH1L[] MEM_TEXT = "RH-";
  const unsigned char RH1H[] MEM_TEXT = "RH+";
  const unsigned char RLRL[] MEM_TEXT = "+RL- 12 13 23";
  const unsigned char RHRH[] MEM_TEXT = "+RH- 12 13 23";
  const unsigned char RHRL[] MEM_TEXT = "RH/RL";
  const unsigned char R0_str[] MEM2_TEXT = "R0=";
  #define LCD_CLEAR
#endif
 
#ifdef CHECK_CALL
  const unsigned char RIHI[] MEM_TEXT = "Ri_Hi=";
  const unsigned char RILO[] MEM_TEXT = "Ri_Lo=";
  const unsigned char C0_str[] MEM_TEXT = "C0 ";
  const unsigned char T50HZ[] MEM_TEXT = " 50Hz";
#endif

#ifdef AUTO_CAL
  const unsigned char MinCap_str[] MEM2_TEXT = " >100nF";
  const unsigned char REF_C_str[] MEM2_TEXT = "REF_C=";
  const unsigned char REF_R_str[] MEM2_TEXT = "REF_R=";
#endif

#ifdef DebugOut
  #define LCD_CLEAR
#endif

#ifdef LCD1602
  const unsigned char DiodeIcon1[] MEM_TEXT = { 0x11, 0x19, 0x1d, 0x1f, 0x1d, 0x19, 0x11, 0x00 };
  const unsigned char DiodeIcon2[] MEM_TEXT = { 0x11, 0x13, 0x17, 0x1f, 0x17, 0x13, 0x11, 0x00 };
  const unsigned char CapIcon[]    MEM_TEXT = { 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x00 };
  const unsigned char ResIcon1[]   MEM_TEXT = { 0x00, 0x0f, 0x08, 0x18, 0x08, 0x0f, 0x00, 0x00 };
  const unsigned char ResIcon2[]   MEM_TEXT = { 0x00, 0x1e, 0x02, 0x03, 0x02, 0x1e, 0x00, 0x00 };
  const unsigned char OmegaIcon[]  MEM_TEXT = { 0x00, 0x00, 0x0e, 0x11, 0x11, 0x0a, 0x1b, 0x00 };
  const unsigned char MicroIcon[]  MEM_TEXT = { 0x00, 0x00, 0x0a, 0x0a, 0x0a, 0x0e, 0x09, 0x10 };
#endif

const unsigned char PinRLtab[] PROGMEM = { (1<<(TP1*2)), (1<<(TP2*2)), (1<<(TP3*2))};
const unsigned char PinADCtab[] PROGMEM = { (1<<TP1), (1<<TP2), (1<<TP3)};

#ifdef AUTO_CAL
  const int8_t RefDiff EEMEM = REF_R_KORR;
#endif

const uint8_t PrefixTab[] MEM_TEXT = { 'p','n',LCD_CHAR_U,'m',0,'k','M'};

#ifdef AUTO_CAL
  const int16_t ref_offset EEMEM = REF_C_KORR;
  const uint8_t c_zero_tab[] EEMEM = { C_NULL,C_NULL,C_NULL+TP2_CAP_OFFSET,C_NULL,C_NULL+TP2_CAP_OFFSET,C_NULL,C_NULL };
#endif

const uint8_t EE_ESR_ZEROtab[] PROGMEM = {ESR_ZERO, ESR_ZERO, ESR_ZERO, ESR_ZERO};

unsigned int RHmultip = DEFAULT_RH_FAKT;

#else
  #define COMMON extern
#endif  // MAIN_C

struct Diode_t {
  uint8_t Anode;
  uint8_t Cathode;
  unsigned int Voltage;
};

COMMON struct Diode_t diodes[6];
COMMON uint8_t NumOfDiodes;

COMMON struct {
  unsigned long hfe[2];		// current amplification factor 
  unsigned int uBE[2];		// B-E-voltage of the Transistor
  uint8_t b,c,e;		// pins of the Transistor
}trans;

COMMON unsigned int gthvoltage;	// Gate-threshold voltage 

COMMON uint8_t PartReady;	// part detection is finished 
COMMON uint8_t PartMode;
COMMON uint8_t tmpval, tmpval2;
COMMON unsigned int ref_mv;     // Reference-voltage in mV units

COMMON struct resis_t{
  unsigned long rx;		// value of resistor RX  
  #if FLASHEND > 0x1fff
    unsigned long lx;		// inductance 10uH or 100uH
    int8_t lpre;		// prefix for inductance
  #endif
  uint8_t ra,rb;		// Pins of RX
  uint8_t rt;			// Tristate-Pin (inactive)
} resis[3];

COMMON uint8_t ResistorsFound;	// Number of found resistors
COMMON uint8_t ii;		// multipurpose counter

COMMON struct cap_t {
  unsigned long cval;		// capacitor value 
  unsigned long cval_max;	// capacitor with maximum value
  union t_combi{
  unsigned long dw;		// capacity value without corrections
  uint16_t w[2];
  } cval_uncorrected;
  #if FLASHEND > 0x1fff
    unsigned int esr;		// serial resistance of C in 0.01 Ohm
    unsigned int v_loss;	// voltage loss 0.1%
  #endif
  uint8_t ca, cb;		// pins of capacitor
  int8_t cpre;			// Prefix for capacitor value  -12=p, -9=n, -6=u, -3=m
  int8_t cpre_max;		// Prefix of the biggest capacitor
} cap;

#ifndef INHIBIT_SLEEP_MODE
  COMMON volatile uint16_t ovcnt16;
  COMMON volatile uint8_t unfinished;
#endif

COMMON int16_t load_diff;	// difference voltage of loaded capacitor and internal reference

COMMON uint8_t WithReference;	// Marker for found precision voltage reference = 1
COMMON uint8_t PartFound;	// the found part 
COMMON char outval[12];		// String for ASCII-output
COMMON uint8_t empty_count;	// counter for max count of empty measurements
COMMON uint8_t mess_count;	// counter for max count of nonempty measurements

COMMON struct ADCconfig_t {
  uint8_t Samples;		// number of ADC samples to take
  uint8_t RefFlag;		// save Reference type VCC of IntRef
  uint16_t U_Bandgap;		// Reference Voltage in mV
  uint16_t U_AVCC;		// Voltage of AVCC
} ADCconfig;

#ifdef AUTO_CAL
  COMMON uint8_t pin_combination;
  COMMON uint16_t resis680pl;
  COMMON uint16_t resis680mi;
  COMMON uint16_t pin_rmi;
  COMMON uint16_t pin_rpl;
#endif

#if POWER_OFF+0 > 1
  COMMON unsigned int display_time;
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// SECTION: Enumerated component categories and semiconductor subtypes.
#define PART_NONE 0
#define PART_DIODE 1
#define PART_TRANSISTOR 2
#define PART_FET 3
#define PART_TRIAC 4
#define PART_THYRISTOR 5
#define PART_RESISTOR 6
#define PART_CAPACITOR 7
#define PART_CELL 8
// UI-level specialization IDs for symbol/screen rendering
#define PART_INDUCTOR 9
#define PART_LED 10

// FETs
#define PART_MODE_N_E_MOS 2
#define PART_MODE_P_E_MOS 3
#define PART_MODE_N_D_MOS 4
#define PART_MODE_P_D_MOS 5
#define PART_MODE_N_JFET 6
#define PART_MODE_P_JFET 7

// Bipolar
#define PART_MODE_NPN 1
#define PART_MODE_PNP 2

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// wait functions
#define  wait5s()    delay(5000)
#define  wait4s()    delay(4000)
#define  wait3s()    delay(3000)
#define  wait2s()    delay(2000)
#define  wait1s()    delay(1000)
#define  wait500ms() delay(500)
#define  wait400ms() delay(400)
#define  wait300ms() delay(300)
#define  wait200ms() delay(200)
#define  wait100ms() delay(100)
#define  wait50ms()  delay(50)
#define  wait40ms()  delay(40)
#define  wait30ms()  delay(30)
#define  wait20ms()  delay(20)
#define  wait10ms()  delay(10)
#define  wait5ms()   delay(5)
#define  wait4ms()   delay(4)
#define  wait3ms()   delay(3)
#define  wait2ms()   delay(2)
#define  wait1ms()   delay(1)
#define  wait500us() delayMicroseconds(500)
#define  wait400us() delayMicroseconds(400)
#define  wait300us() delayMicroseconds(300)
#define  wait250us() delayMicroseconds(250)
#define  wait200us() delayMicroseconds(200)
#define  wait100us() delayMicroseconds(100)
#define  wait50us()  delayMicroseconds(50)
#define  wait40us()  delayMicroseconds(40)
#define  wait30us()  delayMicroseconds(30)
#define  wait20us()  delayMicroseconds(20)
#define  wait10us()  delayMicroseconds(10)
#define  wait5us()   delayMicroseconds(5)
#define  wait4us()   delayMicroseconds(4)
#define  wait3us()   delayMicroseconds(3)
#define  wait2us()   delayMicroseconds(2)
#define  wait1us()   delayMicroseconds(1)

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// LCD-commands
#define CMD_ClearDisplay         0x01
#define CMD_ReturnHome           0x02
#define CMD_SetEntryMode         0x04
#define CMD_SetDisplayAndCursor  0x08
#define CMD_SetIFOptions         0x20
#define CMD_SetCGRAMAddress      0x40
#define CMD_SetDDRAMAddress      0x80

#define lcd_line1() lcd_set_cursor(0,0)
#define lcd_line2() lcd_set_cursor(1,0)
#define lcd_line3() lcd_set_cursor(2,0)
#define lcd_line4() lcd_set_cursor(3,0)

#ifdef WITH_UART
  #define uart_newline() Serial.println()
#else
  #define uart_newline() ((void)0)
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

#ifndef INHIBIT_SLEEP_MODE
  EMPTY_INTERRUPT(TIMER2_COMPA_vect);
  EMPTY_INTERRUPT(ADC_vect);
#endif

uint8_t tmp = 0;

byte TestKey;
byte TestKeyPin = 17;  // A3

#ifdef LCD1602
  #ifdef LCD_I2C
    LiquidCrystal_I2C lcd(0x3F, 16, 2);
  #else
    LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
  #endif
#endif

#ifdef NOK5110
  Adafruit_PCD8544 lcd = Adafruit_PCD8544(3, 4, 5, 6, 7);
#endif

#ifdef OLED096
  #ifdef OLED_I2C
    #define SCREEN_WIDTH 128
    #define SCREEN_HEIGHT 64
    #define OLED_RESET -1
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  #else
    #define OLED_CLK   7
    #define OLED_MOSI  6
    #define OLED_RESET 5
    #define OLED_DC    4
    #define OLED_CS    3
    Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
  #endif
#endif

// Forward declarations for measurement routines invoked by the OLED UI
void DisplayValue(unsigned long Value, int8_t Exponent, unsigned char Unit, unsigned char digits);
void ReadCapacity(uint8_t HighPin, uint8_t LowPin);
uint16_t GetESR(uint8_t hipin, uint8_t lopin);
void GetVloss(void);

/* ===============================================================
 * SECTION: MG_V1.1 OLED USER INTERFACE + RGB STATUS LED
 * Hardware Mapping:
 * - OLED: SSD1306 128x64 I2C (A4 SDA, A5 SCL)
 * - RGB LED: Common Anode (+5V on common pin)
 *   D6 -> Red (Fault / No component / Live voltage)
 *   D5 -> Green (Valid component measured)
 *   D4 -> Blue (Measurement in progress)
 * - Button: A3 with external 10k pull-up to 5V
 * =============================================================== */
#define MG_OLED_TOP_OFFSET 3
#define MG_LED_RED_PIN 6
#define MG_LED_GREEN_PIN 5
#define MG_LED_BLUE_PIN 4
#define MG_RGB_COMMON_ANODE 1 // 0: common cathode at GND; 1: common anode at +5V

#define MG_STATUS_OFF 0
#define MG_STATUS_TESTING 1
#define MG_STATUS_MEASURED 2
#define MG_STATUS_CHECK 3

#ifdef OLED096
// MG Brand Logo Bitmap (128x64 = 1024 bytes stored strictly in PROGMEM)
static const uint8_t logoBitmap[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x80, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xC0, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xC0, 0x00, 0x7F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xE0, 0x00, 0x7F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xF0, 0x00, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xF0, 0x01, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xF8, 0x01, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFC, 0x03, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFC, 0x03, 0xFF, 0xFC, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0x07, 0xFF, 0xFC, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0x0F, 0xFF, 0xFC, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x0F, 0xFF, 0xFC, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x9F, 0xFF, 0xFC, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x9F, 0xFF, 0xFC, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x7F, 0xFF, 0xCF, 0xFC, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x3F, 0xFF, 0xCF, 0xFC, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x3F, 0xFF, 0x8F, 0xFC, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x1F, 0xFF, 0x0F, 0xFC, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x0F, 0xFF, 0x0F, 0xFD, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x0F, 0xFE, 0x0F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x07, 0xFC, 0x0F, 0xFF, 0x07, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x03, 0xFC, 0x0F, 0xFE, 0x0F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x03, 0xF8, 0x0F, 0xFC, 0x1F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x01, 0xF0, 0x0F, 0xF8, 0x3F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0xF0, 0x0F, 0xF0, 0x7F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0xE0, 0x0F, 0xE1, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x60, 0x0F, 0x83, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x0F, 0x07, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x0E, 0x0F, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x0C, 0x1F, 0xFE, 0x1F, 0xF0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x18, 0x3F, 0xF0, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x7F, 0xE0, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x03, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFE, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFE, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE7, 0xFE, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x8F, 0xFE, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0xFE, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x07, 0xFE, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x07, 0xFF, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x07, 0xFF, 0x00, 0x01, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x07, 0xFF, 0x00, 0x01, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x03, 0xFF, 0x80, 0x01, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0xFF, 0xC0, 0x01, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x03, 0xFF, 0xE0, 0x01, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x01, 0xFF, 0xF8, 0x03, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static_assert(
  sizeof(logoBitmap) == SCREEN_WIDTH * SCREEN_HEIGHT / 8,
  "Logo bitmap must contain exactly 1024 bytes"
);

// FUNCTION: mgLed - Set one solid RGB state (active-LOW for Common Anode).
void mgLed(uint8_t state) {
  bool red = state == MG_STATUS_CHECK;
  bool green = state == MG_STATUS_MEASURED;
  bool blue = state == MG_STATUS_TESTING;
  #if MG_RGB_COMMON_ANODE
    digitalWrite(MG_LED_RED_PIN, red ? LOW : HIGH);
    digitalWrite(MG_LED_GREEN_PIN, green ? LOW : HIGH);
    digitalWrite(MG_LED_BLUE_PIN, blue ? LOW : HIGH);
  #else
    digitalWrite(MG_LED_RED_PIN, red ? HIGH : LOW);
    digitalWrite(MG_LED_GREEN_PIN, green ? HIGH : LOW);
    digitalWrite(MG_LED_BLUE_PIN, blue ? HIGH : LOW);
  #endif
}

// Compact helper to save Flash across repeated line draws & cursor jumps
void mgLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
  display.drawLine(x0, y0, x1, y1, WHITE);
}
void mgCur1() { display.setCursor(50, 30); }
void mgCur2() { display.setCursor(50, 42); }
void mgCurB() { display.setCursor(0, 56); }
void mgPrintTP(uint8_t p) {
  display.print(F("TP"));
  display.print(p + 1);
}

// FUNCTION: mgBrandScreen - Display MG Engineer title & component tester description.
void mgBrandScreen() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setTextWrap(false);
  display.setCursor(31, MG_OLED_TOP_OFFSET + 2);
  display.print(F("MG Engineer"));
  mgLine(0, 16, 127, 16);
  display.setCursor(10, 21);
  display.print(F("Component Tester &"));
  display.setCursor(16, 31);
  display.print(F("Measurement Tool"));
  display.setCursor(10, 43);
  display.print(F("Test & measure any"));
  display.setCursor(4, 53);
  display.print(F("electronic component"));
  display.display();
}

// FUNCTION: mgIdleScreen - Display the idle instructions and wait for the physical TEST button.
void mgIdleScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextWrap(false);
  display.setCursor(0, MG_OLED_TOP_OFFSET + 1);
  display.print(F("MG ENGINEER  V1.1"));
  mgLine(0, 15, 127, 15);
  display.setCursor(18, 23);
  display.print(F("COMPONENT TESTER"));
  display.setCursor(20, 39);
  display.print(F("Insert component"));
  display.setCursor(27, 51);
  display.print(F("Press TEST"));
  display.display();
}

// FUNCTION: mgTestScreen - Show the measurement-in-progress screen while driving the blue LED.
void mgTestScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextWrap(false);
  display.setCursor(0, MG_OLED_TOP_OFFSET + 1);
  display.print(F("MG ENGINEER  V1.1"));
  mgLine(0, 15, 127, 15);
  display.setCursor(24, 28);
  display.print(F("TESTING..."));
  display.setCursor(12, 44);
  display.print(F("Checking TP1-TP3"));
  display.display();
}

// FUNCTION: mgDrawSymbol - Render a compact schematic icon for the detected component type.
void mgDrawSymbol(uint8_t part) {
  if (part == PART_RESISTOR) {
    mgLine(3, 34, 13, 34);
    display.drawRect(13, 29, 21, 11, WHITE);
    mgLine(34, 34, 45, 34);
  } else if (part == PART_INDUCTOR) {
    mgLine(3, 34, 9, 34);
    for (uint8_t k = 0; k < 4; k++) {
      uint8_t x0 = 9 + k * 7;
      mgLine(x0, 34, x0 + 1, 29);
      mgLine(x0 + 1, 29, x0 + 5, 29);
      mgLine(x0 + 5, 29, x0 + 6, 34);
    }
    mgLine(37, 34, 45, 34);
  } else if (part == PART_CAPACITOR) {
    mgLine(3, 34, 17, 34);
    mgLine(18, 24, 18, 44);
    mgLine(29, 24, 29, 44);
    mgLine(30, 34, 45, 34);
  } else if (part == PART_DIODE) {
    mgLine(3, 34, 15, 34);
    display.drawTriangle(15, 26, 15, 42, 28, 34, WHITE);
    mgLine(29, 24, 29, 44);
    mgLine(29, 34, 45, 34);
  } else if (part == PART_LED) {
    mgLine(3, 37, 15, 37);
    display.drawTriangle(15, 29, 15, 45, 28, 37, WHITE);
    mgLine(29, 27, 29, 47);
    mgLine(29, 37, 45, 37);
    mgLine(22, 27, 29, 20);
    mgLine(25, 20, 29, 20);
    mgLine(29, 20, 29, 24);
    mgLine(28, 29, 35, 22);
    mgLine(31, 22, 35, 22);
    mgLine(35, 22, 35, 26);
  } else if (part == PART_TRANSISTOR) {
    mgLine(4, 34, 18, 34);
    mgLine(18, 24, 18, 44);
    mgLine(18, 28, 36, 19);
    mgLine(36, 19, 43, 19);
    mgLine(18, 40, 36, 49);
    mgLine(36, 49, 43, 49);
    mgLine(27, 40, 34, 43);
    mgLine(34, 43, 29, 45);
  } else if (part == PART_FET) {
    mgLine(7, 27, 7, 43);
    mgLine(7, 35, 19, 35);
    mgLine(24, 25, 24, 43);
    mgLine(24, 25, 43, 25);
    mgLine(24, 43, 43, 43);
    mgLine(19, 35, 21, 35);
  } else if (part == PART_TRIAC) {
    mgLine(3, 34, 13, 34);
    mgLine(13, 28, 13, 40);
    display.drawTriangle(13, 23, 13, 33, 29, 28, WHITE);
    display.drawTriangle(29, 35, 29, 45, 13, 40, WHITE);
    mgLine(29, 23, 29, 45);
    mgLine(29, 34, 44, 34);
    mgLine(13, 42, 7, 48);
    mgLine(3, 48, 7, 48);
  } else if (part == PART_THYRISTOR) {
    mgLine(4, 34, 15, 34);
    display.drawTriangle(15, 27, 15, 41, 28, 34, WHITE);
    mgLine(29, 25, 29, 43);
    mgLine(29, 34, 43, 34);
    mgLine(18, 41, 10, 49);
  } else {
    display.drawCircle(23, 34, 13, WHITE);
    display.setCursor(20, 30);
    display.print('?');
  }
}

// FUNCTION: mgPinout - Convert the measured pin indexes into a human-readable 123 pin map.
void mgPinout(char a, char b, char c) {
  display.print(F("123="));
  for (uint8_t p=0;p<3;p++) {
    display.print(p==trans.e ? a : (p==trans.b ? b : (p==trans.c ? c : '?')));
  }
}

// FUNCTION: mgResultScreen - Present final measurements and drive Red D6 on fault/no part.
void mgResultScreen() {
  bool isInductor = (PartFound == PART_RESISTOR && ResistorsFound == 1 && resis[0].lx > 0);
  bool isLed = (PartFound == PART_DIODE && NumOfDiodes == 1 && diodes[0].Voltage >= 1350);

  // Measure MOSFET gate capacitance before rendering so Cg is accurate
  if (PartFound == PART_FET && PartMode < PART_MODE_N_D_MOS) {
    ReadCapacity(trans.b, trans.e);
  }
  // Measure single Diode / LED reverse junction capacitance
  if (PartFound == PART_DIODE && NumOfDiodes == 1) {
    ReadCapacity(diodes[0].Cathode, diodes[0].Anode);
  }

  // Evaluate component health / fault conditions
  bool isFaulty = false;
  if (PartFound == PART_NONE || PartFound == PART_CELL) {
    isFaulty = true;
  } else if (PartFound == PART_RESISTOR && ResistorsFound == 1 && resis[0].rx == 0 && resis[0].lx == 0) {
    isFaulty = true; // Dead short (< 0.1 Ohm)
  } else if (PartFound == PART_CAPACITOR && (cap.v_loss > 150 || (cap.esr < 65530 && cap.esr > 5000))) {
    isFaulty = true; // Excessive self-discharge loss (>15%) or severe ESR (>50 Ohm)
  } else if (PartFound == PART_TRANSISTOR && (trans.hfe[0] < 5 || trans.uBE[0] < 100 || trans.uBE[0] > 2500)) {
    isFaulty = true; // Damaged / open / shorted BJT junction
  }

  // Red LED (D6) lights up on faulty or undetected component; Green (D5) when valid
  mgLed(isFaulty ? MG_STATUS_CHECK : MG_STATUS_MEASURED);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setTextWrap(false);
  display.setCursor(0,MG_OLED_TOP_OFFSET + 1);
  display.print(isFaulty ? F("MG_V1.1  /  FAULT!") : F("MG_V1.1  /  RESULT"));
  mgLine(0,15,127,15);

  uint8_t symbolType = isInductor ? PART_INDUCTOR : (isLed ? PART_LED : PartFound);
  mgDrawSymbol(symbolType);
  display.setCursor(50,19);

  if (isInductor) {
    display.print(F("INDUCTOR"));
    mgCur1();
    display.print(F("L="));
    DisplayValue(resis[0].lx, resis[0].lpre, 'H', 4);
    mgCur2();
    display.print(F("R="));
    if (resis[0].rx < 200) {
      DisplayValue(GetESR(resis[0].ra, resis[0].rb), -2, 0, 3);
    } else {
      DisplayValue(resis[0].rx, -1, 0, 4);
    }
    display.print(F(" Ohm"));
    mgCurB();
    mgPrintTP(resis[0].rb);
    display.print(F(" <-> ")); mgPrintTP(resis[0].ra);
  } else if (PartFound == PART_RESISTOR) {
    display.print(isFaulty ? F("SHORTED") : (ResistorsFound <= 1 ? F("RESISTOR") : F("R NETWORK")));
    mgCur1();
    display.print(ResistorsFound > 1 ? F("R1=") : F("R="));
    if (ResistorsFound == 1 && resis[0].rx < 200) {
      DisplayValue(GetESR(resis[0].ra, resis[0].rb), -2, 0, 3);
    } else {
      DisplayValue(resis[0].rx, -1, 0, 4);
    }
    mgCur2();
    if (ResistorsFound > 1) {
      display.print(F("R2="));
      DisplayValue(resis[1].rx,-1,0,3);
    } else {
      display.print(F("Ohms"));
    }
    mgCurB();
    if (ResistorsFound <= 1) {
      mgPrintTP(resis[0].rb);
      display.print(F(" <-> ")); mgPrintTP(resis[0].ra);
    } else if (ResistorsFound >= 3) {
      display.print(F("R3="));
      DisplayValue(resis[2].rx,-1,0,4);
    } else {
      display.print(F("2 resistive paths"));
    }
  } else if (PartFound == PART_CAPACITOR) {
    display.print(isFaulty ? F("BAD CAP") : F("CAPACITOR"));
    mgCur1();
    display.print(F("C="));
    DisplayValue(cap.cval_max,cap.cpre_max,'F',4);
    mgCur2();
    if (cap.esr < 65530) {
      display.print(F("ESR="));
      DisplayValue(cap.esr,-2,0,2);
    } else {
      display.print(F("ESR: N/A"));
    }
    mgCurB();
    mgPrintTP(cap.ca);
    display.print(F(" - ")); mgPrintTP(cap.cb);
    if (cap.v_loss) {
      display.print(F(" Loss="));
      DisplayValue(cap.v_loss,-1,'%',2);
    }
  } else if (isLed) {
    display.print(F("LED"));
    mgCur1();
    display.print(F("Vf="));
    DisplayValue(diodes[0].Voltage,-3,'V',3);
    mgCur2();
    display.print(F("C="));
    DisplayValue(cap.cval,cap.cpre,'F',3);
    mgCurB();
    display.print(F("A=")); mgPrintTP(diodes[0].Anode);
    display.print(F("  K=")); mgPrintTP(diodes[0].Cathode);
  } else if (PartFound == PART_DIODE) {
    display.print(NumOfDiodes > 1 ? F("2x DIODE") : F("DIODE"));
    mgCur1();
    if (NumOfDiodes == 1) {
      display.print(F("Vf="));
      DisplayValue(diodes[0].Voltage,-3,'V',3);
      mgCur2();
      display.print(F("C="));
      DisplayValue(cap.cval,cap.cpre,'F',3);
    } else {
      display.print(F("Count=")); display.print(NumOfDiodes);
      mgCur2();
      display.print(F("Vf="));
      DisplayValue(diodes[0].Voltage,-3,'V',3);
    }
    mgCurB();
    if (NumOfDiodes == 1) {
      display.print(F("A=")); mgPrintTP(diodes[0].Anode);
      display.print(F("  K=")); mgPrintTP(diodes[0].Cathode);
    } else if (NumOfDiodes >= 2) {
      display.print(F("D2 Vf="));
      DisplayValue(diodes[1].Voltage,-3,'V',3);
    }
  } else if (PartFound == PART_TRANSISTOR) {
    display.print(PartMode==PART_MODE_NPN ? F("NPN BJT") : F("PNP BJT"));
    mgCur1();
    display.print(F("hFE=")); display.print(trans.hfe[0]);
    mgCur2();
    display.print(F("Vbe="));
    DisplayValue(trans.uBE[0],-3,'V',3);
    mgCurB();
    mgPinout('E','B','C');
  } else if (PartFound == PART_FET) {
    bool pChannel = PartMode & 1;
    bool isJfet = (PartMode >= PART_MODE_N_JFET);
    display.print(pChannel ? F("P-") : F("N-"));
    display.print(isJfet ? F("JFET") : F("MOSFET"));
    mgCur1();
    display.print(F("Vt="));
    DisplayValue(gthvoltage,-3,'V',2);
    mgCur2();
    if (PartMode < PART_MODE_N_D_MOS) {
      display.print(F("Cg="));
      DisplayValue(cap.cval,cap.cpre,'F',3);
    } else {
      display.print(F("D/J type"));
    }
    mgCurB();
    mgPinout('S','G','D');
  } else if (PartFound == PART_TRIAC) {
    display.print(F("TRIAC"));
    mgCur1();
    display.print(F("AC Switch"));
    mgCur2();
    display.print(F("Detected"));
    mgCurB();
    mgPinout('1','G','2');
  } else if (PartFound == PART_THYRISTOR) {
    display.print(F("SCR"));
    mgCur1();
    display.print(F("Thyristor"));
    mgCur2();
    display.print(F("Detected"));
    mgCurB();
    mgPinout('C','G','A');
  } else if (PartFound == PART_CELL) {
    display.print(F("LIVE INPUT"));
    mgCur1();
    display.print(F("STOP"));
    mgCur2();
    display.print(F("Check DUT"));
    mgCurB();
    display.print(F("Remove applied voltage"));
  } else {
    display.print(F("NO / FAULTY"));
    mgCur1();
    display.print(F("No part or"));
    mgCur2();
    display.print(F("Damaged"));
    mgCurB();
    display.print(F("Check TP1-TP3 leads"));
  }
  display.display();
}
#endif // OLED096

// SECTION: Main setup/measurement state machine.
// FUNCTION: setup - Configure inputs, OLED, LEDs and ADC, then display MG logo & brand intro.
void setup()
{
  #ifdef WITH_UART
    Serial.begin(9600);
  #endif

  pinMode(TestKeyPin, INPUT);
  #ifdef OLED096
    pinMode(MG_LED_RED_PIN, OUTPUT);
    pinMode(MG_LED_GREEN_PIN, OUTPUT);
    pinMode(MG_LED_BLUE_PIN, OUTPUT);
    mgLed(MG_STATUS_OFF);
  #endif

  #ifdef LCD1602
    #ifdef LCD_I2C
      lcd.begin();
    #else
      lcd.begin(16,2);
    #endif

    lcd_pgm_custom_char(LCD_CHAR_DIODE1, DiodeIcon1);
    lcd_pgm_custom_char(LCD_CHAR_DIODE2, DiodeIcon2);
    lcd_pgm_custom_char(LCD_CHAR_CAP,    CapIcon);
    lcd_pgm_custom_char(LCD_CHAR_RESIS1, ResIcon1);
    lcd_pgm_custom_char(LCD_CHAR_RESIS2, ResIcon2);
    lcd_pgm_custom_char(LCD_CHAR_OMEGA,  OmegaIcon);
    lcd_pgm_custom_char(LCD_CHAR_U,      MicroIcon);
    lcd.home();
  
    lcd_string("MG Engineer");
    lcd_set_cursor(1, 0);
    lcd_string("Tester V1.1");
  #endif

  #ifdef NOK5110
    lcd.begin();
    lcd.cp437(true);
    lcd.setContrast(40);
    lcd.clearDisplay();
  #endif

  #ifdef OLED096
    #ifdef OLED_I2C
      if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        mgLed(MG_STATUS_CHECK);
        while (true) { delay(100); }
      }
    #else
      display.begin(SSD1306_SWITCHCAPVCC);
    #endif

    display.cp437(true);
    display.invertDisplay(false);

    // 1) Display MG Logo Bitmap
    display.clearDisplay();
    display.drawBitmap(0, 0, logoBitmap, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
    display.display();
    delay(2000);

    // 2) Display MG Engineer & Component Tester Description
    mgBrandScreen();
    delay(2200);

    // 3) Enter normal Component Tester Ready Screen
    mgIdleScreen();
  #endif

  #ifdef NOK5110
    lcd_string("MG Engineer");
    lcd_set_cursor(1, 0);
    lcd_string("Component");
    lcd_set_cursor(2, 0);
    lcd_string("Tester");
    lcd_set_cursor(3, 0);
    lcd_string("MG_V1.1");
  #endif

  // ADC-Init
  ADCSRA = (1<<ADEN) | AUTO_CLOCK_DIV;

  #ifdef __AVR_ATmega8__
    #define WDRF_HOME MCUCSR
  #else
    #define WDRF_HOME MCUSR
  #endif

  #define T2_PERIOD (1024 / (F_CPU / 1000000UL));

  if(tmp) { 
    lcd_line1();
    lcd_fix_string(TestTimedOut);
    wait_about3s();
  }

  #ifdef PULLUP_DISABLE
    #ifdef __AVR_ATmega8__
      SFIOR = (1<<PUD);
    #else
      MCUCR = (1<<PUD);
    #endif
  #endif

  #define display_time OFF_WAIT_TIME

  empty_count = 0;
  mess_count = 0;
}

// FUNCTION: loop - Handle one button-triggered measurement using the test engine.
void loop()
{
start:

  #ifdef NOK5110
    lcd.display();
  #endif

  TestKey = 1;
  while(TestKey) {
    TestKey = digitalRead(TestKeyPin);
    delay(100);
  }
  while(!TestKey) {
    TestKey = digitalRead(TestKeyPin);
    delay(100);
  }
  lcd_clear();
  #ifdef OLED096
    mgLed(MG_STATUS_TESTING);
    mgTestScreen();
  #endif
  delay(100);

  PartFound = PART_NONE;
  NumOfDiodes = 0;
  PartReady = 0;
  PartMode = 0;
  WithReference = 0;
  ADC_DDR = TXD_MSK;
  ResistorsFound = 0;
  #if FLASHEND > 0x1fff
    for (uint8_t mi = 0; mi < 3; mi++) {
      resis[mi].lx = 0;
      resis[mi].lpre = -5;
    }
    cap.esr = 65535;
    cap.v_loss = 0;
  #endif
  cap.ca = 0;
  cap.cb = 0;

  #ifdef WITH_UART
    uart_newline();
  #endif

  ADCconfig.RefFlag = 0;
  Calibrate_UR();
  ADCconfig.U_Bandgap = ADC_internal_reference;

  #ifndef OLED096
    lcd_line1();
    #ifdef BAT_CHECK
      ReadADC(TPBAT);
      trans.uBE[0] = W5msReadADC(TPBAT);
      lcd_fix_string(Bat_str);

      #ifdef BAT_OUT
        cap.cval = (trans.uBE[0]*4)+BAT_OUT;
        DisplayValue(cap.cval,-3,'V',2);
        lcd_space();
      #endif

      #if (BAT_POOR > 5300)
        #define WARN_LEVEL (((unsigned long)(BAT_POOR+800)*(unsigned long)33)/133)
      #elif (BAT_POOR > 3249)
        #define WARN_LEVEL (((unsigned long)(BAT_POOR+400)*(unsigned long)33)/133)
      #elif (BAT_POOR > 1299)
        #define WARN_LEVEL (((unsigned long)(BAT_POOR+200)*(unsigned long)33)/133)
      #else
        #define WARN_LEVEL (((unsigned long)(BAT_POOR+100)*(unsigned long)33)/133)
      #endif

      #define POOR_LEVEL (((unsigned long)(BAT_POOR)*(unsigned long)33)/133)

      if (trans.uBE[0] < WARN_LEVEL) {
        if(trans.uBE[0] < POOR_LEVEL) {
          lcd_fix_string(BatEmpty);
          wait_about2s();
          PORTD = 0;
          return;
        }
        lcd_fix_string(BatWeak);
      } else {
        lcd_fix_string(OK_str);
      }
    #else
      lcd_fix2_string(VERSION_str);
    #endif
  #endif // !OLED096

  #ifdef AUTO_RH
    RefVoltage();
  #endif

  #ifndef OLED096
    #if FLASHEND > 0x1fff
      if (WithReference) {
        if ((mess_count == 0) && (empty_count == 0)) {
          lcd_line2();
          lcd_fix_string(VCC_str);
          DisplayValue(ADCconfig.U_AVCC,-3,'V',3);
          wait_about1s();
        }
      }
    #endif

    #ifdef WITH_VEXT
      while (!(ON_PIN_REG & (1<<RST_PIN))) {
        lcd_line2();
        lcd_clear_line();
        lcd_line2();
        lcd_fix_string(Vext_str);
        ADC_DDR = 0;
        trans.uBE[1] = W5msReadADC(TPext);
        ADC_DDR = TXD_MSK;

        #ifdef WITH_UART
          uart_newline();
        #endif

        DisplayValue(trans.uBE[1]*10,-3,'V',3);
        wait_about300ms();
      }
    #endif

    lcd_line2();
    lcd_fix_string(TestRunning);

    #ifndef DebugOut
      lcd_line2();
    #endif

    #ifdef NOK5110
      lcd.display();
    #endif
  #endif // !OLED096

  delay(5);

  EntladePins();

  if(PartFound == PART_CELL) {
    #ifndef OLED096
      lcd_clear();
      lcd_fix_string(Cell_str);
    #endif
    goto end2;
  }

  #ifdef CHECK_CALL
    AutoCheck();
  #endif
     
  // check all 6 combinations for the 3 pins 
  CheckPins(TP1, TP2, TP3);
  CheckPins(TP2, TP1, TP3);
  CheckPins(TP1, TP3, TP2);
  CheckPins(TP3, TP1, TP2);
  CheckPins(TP2, TP3, TP1);
  CheckPins(TP3, TP2, TP1);
  
  // separate check if it is a capacitor or inductor
  if(((PartFound == PART_NONE) || (PartFound == PART_RESISTOR) || (PartFound == PART_DIODE)) ) {
    EntladePins();
    cap.cval_max = 0;
    cap.cpre_max = -12;

    ReadCapacity(TP3, TP1);
    ReadCapacity(TP3, TP2);
    ReadCapacity(TP2, TP1);

    #if FLASHEND > 0x1fff
      ReadInductance();
    #endif
  }

  // Finalize BJT emitter/collector pin orientation when dual-pass detected
  if (PartFound == PART_TRANSISTOR) {
    if(PartReady != 0) {
      if((trans.hfe[0]>trans.hfe[1])) {
        tmp = trans.c;
        trans.c = trans.e;
        trans.e = tmp;
      } else {
        trans.hfe[0] = trans.hfe[1];
        trans.uBE[0] = trans.uBE[1];
      }
    }
  }

#ifdef OLED096
  // Populate capacitor Vloss and ESR before graphical OLED rendering
  #if FLASHEND > 0x1fff
    if (PartFound == PART_CAPACITOR) {
      GetVloss();
      cap.esr = GetESR(cap.cb, cap.ca);
    }
  #endif

  if (PartFound != PART_NONE) {
    empty_count = 0;
    mess_count++;
  } else {
    empty_count++;
    mess_count = 0;
  }
  goto end2;

#else
  // Legacy character/NOK5110 display output (excluded in OLED096 build to save Flash)
  lcd_clear();

  if(PartFound == PART_DIODE) {
    if(NumOfDiodes == 1) {
      #if FLASHEND > 0x1fff
        #if EBC_STYLE == 321
          if (diodes[0].Anode > diodes[0].Cathode) {
            lcd_testpin(diodes[0].Anode);
            lcd_fix_string(AnKat);
            lcd_testpin(diodes[0].Cathode);
          } else {
            lcd_testpin(diodes[0].Cathode);
            lcd_fix_string(KatAn);
            lcd_testpin(diodes[0].Anode);
          }
        #else
          if (diodes[0].Anode < diodes[0].Cathode) {
            lcd_testpin(diodes[0].Anode);
            lcd_fix_string(AnKat);
            lcd_testpin(diodes[0].Cathode);
          } else {
            lcd_testpin(diodes[0].Cathode);
            lcd_fix_string(KatAn);
            lcd_testpin(diodes[0].Anode);
          }
        #endif
      #else
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);
        lcd_testpin(diodes[0].Cathode);
      #endif

      #if FLASHEND > 0x1fff
        GetIr(diodes[0].Cathode,diodes[0].Anode);
      #endif

      UfOutput(0x70);

      #if defined(NOK5110)
        lcd_line3();
      #endif

      lcd_fix_string(GateCap_str);
      ReadCapacity(diodes[0].Cathode,diodes[0].Anode);
      DisplayValue(cap.cval,cap.cpre,'F',3);
      goto end;

    } else if(NumOfDiodes == 2) {
      lcd_data('2');
      lcd_fix_string(Diodes);

      if(diodes[0].Anode == diodes[1].Anode) {
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(KatAn);
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);
        lcd_testpin(diodes[1].Cathode);
        UfOutput(0x01);
        goto end;

      } else if(diodes[0].Cathode == diodes[1].Cathode) {
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(KatAn);
        lcd_testpin(diodes[1].Anode);
        UfOutput(0x01);
        goto end;

      } else if ((diodes[0].Cathode == diodes[1].Anode) && (diodes[1].Cathode == diodes[0].Anode)) {
        lcd_testpin(diodes[0].Anode);
        lcd_fix_string(AnKat);
        lcd_testpin(diodes[0].Cathode);
        lcd_fix_string(AnKat);
        lcd_testpin(diodes[1].Cathode);
        UfOutput(0x01);
        goto end;
      }

    } else if(NumOfDiodes == 3) {
      trans.b = 3;
      trans.c = 3;

      if(diodes[0].Cathode == diodes[1].Anode) {
        trans.b = 0;
        trans.c = 1;
      }
      if(diodes[0].Anode == diodes[1].Cathode) {
        trans.b = 1;
        trans.c = 0;
      }
      if(diodes[0].Cathode == diodes[2].Anode) {
        trans.b = 0;
        trans.c = 2;
      }
      if(diodes[0].Anode == diodes[2].Cathode) {
        trans.b = 2;
        trans.c = 0;
      }
      if(diodes[1].Cathode == diodes[2].Anode) {
        trans.b = 1;
        trans.c = 2;
      }
      if(diodes[1].Anode == diodes[2].Cathode) {
        trans.b = 2;
        trans.c = 1;
      }

      if((trans.b < 3) && (trans.c < 3)) {
        lcd_data('3');
        lcd_fix_string(Diodes);
        lcd_testpin(diodes[trans.b].Anode);
        lcd_fix_string(AnKat);
        lcd_testpin(diodes[trans.b].Cathode);
        lcd_fix_string(AnKat);
        lcd_testpin(diodes[trans.c].Cathode);
        UfOutput((trans.b<<4)|trans.c);
        goto end;
      }
    }

  } else if (PartFound == PART_TRANSISTOR) {
    if(PartMode == PART_MODE_NPN) {
      lcd_fix_string(NPN_str);
    } else {
      lcd_fix_string(PNP_str);
    }

    if(NumOfDiodes > 2) {
      #ifdef EBC_STYLE
        #if EBC_STYLE == 321
          if (((PartMode == PART_MODE_NPN) && (trans.c < trans.e)) || ((PartMode != PART_MODE_NPN) && (trans.c > trans.e)))
        #else
          if(PartMode == PART_MODE_NPN)
        #endif
      #else
        if (((PartMode == PART_MODE_NPN) && (trans.c > trans.e)) || ((PartMode != PART_MODE_NPN) && (trans.c < trans.e)))
      #endif
        {
          lcd_fix_string(AnKat);
        } else {
          lcd_fix_string(KatAn);
        }
    }

    #if defined(NOK5110)
      lcd_line2();
    #endif

    PinLayout('E','B','C');

    #if defined(NOK5110)
      lcd_line3();
    #else
      lcd_line2();
    #endif

    lcd_fix_string(hfe_str);
    DisplayValue(trans.hfe[0],0,0,3);
    lcd_space();

    #if defined(NOK5110)
      lcd_line4();
    #endif

    lcd_fix_string(Uf_str);
    DisplayValue(trans.uBE[0],-3,'V',3);
    goto end;

  } else if (PartFound == PART_FET) {
    if(PartMode&1) {
      lcd_data('P');
    } else {
      lcd_data('N');
    }
    lcd_data('-');

    tmp = PartMode/2;
    if (tmp == (PART_MODE_N_D_MOS/2)) {
      lcd_data('D');
    }
    if (tmp == (PART_MODE_N_E_MOS/2)) {
      lcd_data('E');
    }

    if (tmp == (PART_MODE_N_JFET/2)) {
      lcd_fix_string(jfet_str);
    } else {
      lcd_fix_string(mosfet_str);
    }

    #if defined(NOK5110)
      lcd_line2();
    #endif

    PinLayout('S','G','D');

    if((NumOfDiodes > 0) && (PartMode < PART_MODE_N_D_MOS)) {
      #ifdef EBC_STYLE
        #if EBC_STYLE == 321
          if (((PartMode&1) && (trans.c > trans.e)) || ((!(PartMode&1)) && (trans.c < trans.e)))
        #else
          if (PartMode&1)
        #endif
      #else
        if (((PartMode&1) && (trans.c < trans.e)) || ((!(PartMode&1)) && (trans.c > trans.e)))
      #endif
        {
          lcd_data(LCD_CHAR_DIODE1);
        } else {
          lcd_data(LCD_CHAR_DIODE2);
        }
    }

    #if defined(NOK5110)
      lcd_line3();
    #else
      lcd_line2();
    #endif

    if(PartMode < PART_MODE_N_D_MOS) {
      lcd_fix_string(GateCap_str);
      ReadCapacity(trans.b,trans.e);
      DisplayValue(cap.cval,cap.cpre,'F',3);

      #if defined(NOK5110)
        lcd_line4();
      #endif

      lcd_fix_string(vt_str);
    } else {
      lcd_data('I');
      lcd_data('=');
      DisplayValue(trans.uBE[1],-5,'A',2);

      #if defined(NOK5110)
        lcd_line4();
      #endif

      lcd_fix_string(Vgs_str);
    }

    DisplayValue(gthvoltage,-3,'V',2);
    goto end;

  } else if (PartFound == PART_THYRISTOR) {
    lcd_fix_string(Thyristor);
    goto gakOutput;

  } else if (PartFound == PART_TRIAC) {
    lcd_fix_string(Triac);
    goto gakOutput;

  } else if(PartFound == PART_RESISTOR) {
    if (ResistorsFound == 1) {
      lcd_testpin(resis[0].rb);
      lcd_fix_string(Resistor_str);
      lcd_testpin(resis[0].ra);
    } else {
      ii = 0;
      if (resis[1].rx > resis[0].rx)
        ii = 1;

      if (ResistorsFound == 2) {
        ii = 2;
      } else {
        if (resis[2].rx > resis[ii].rx)
          ii = 2;
      }

      char x = '1';
      char y = '3';
      char z = '2';
   
      if (ii == 1) {
        y = '2';
        z = '3';
      }

      if (ii == 2) {
        x = '2';
        y = '1';
        z = '3';
      }

      lcd_data(x);
      lcd_fix_string(Resistor_str);
      lcd_data(y);
      lcd_fix_string(Resistor_str);
      lcd_data(z);
    }

    lcd_line2();

    if (ResistorsFound == 1) {
      RvalOut(0);

      #if FLASHEND > 0x1fff
        if (resis[0].lx != 0) {
          #if defined(NOK5110)
            lcd_line3();
          #endif

          lcd_fix_string(Lis_str);
          DisplayValue(resis[0].lx,resis[0].lpre,'H',3);
        }
      #endif

    } else {
      if (ii == 0) {
        RvalOut(1);
        RvalOut(2);
      }
      if (ii == 1) {
        RvalOut(0);
        RvalOut(2);
      }
      if (ii == 2) {
        RvalOut(0);
        RvalOut(1);
      }
    }
    goto end;

  } else if(PartFound == PART_CAPACITOR) {
    lcd_testpin(cap.ca);
    lcd_fix_string(CapZeich);
    lcd_testpin(cap.cb);

    #if FLASHEND > 0x1fff
      GetVloss();
      if (cap.v_loss != 0) {
        #if defined(NOK5110)
          lcd_line4();
        #endif

        lcd_fix_string(VLOSS_str);
        DisplayValue(cap.v_loss,-1,'%',2);
      }
    #endif

    lcd_line2();
    DisplayValue(cap.cval_max,cap.cpre_max,'F',4);

    #if FLASHEND > 0x1fff
      cap.esr = GetESR(cap.cb, cap.ca);
      if (cap.esr < 65530) {
        #if defined(NOK5110)
          lcd_line3();
        #endif

        lcd_fix_string(ESR_str);
        DisplayValue(cap.esr,-2,LCD_CHAR_OMEGA,2);
      }
    #endif

    goto end;
  }

  if(NumOfDiodes == 0) {
    lcd_fix_string(TestFailed1);
    lcd_line2();
    lcd_fix_string(TestFailed2);
    lcd_fix_string(Component);
  } else {
    lcd_fix_string(Component);
    lcd_fix_string(Unknown);
    lcd_line2();
    lcd_fix_string(OrBroken);
    lcd_data(NumOfDiodes + '0');
    lcd_fix_string(AnKat);
  }

  empty_count++;
  mess_count = 0;
  goto end2;

gakOutput:
  lcd_line2();
  PinLayout(Cathode_char,'G','A');

//- - - - - - - - - - - - - - - - - - - - - - - - - - - -
end:
  empty_count = 0;
  mess_count++;
#endif // OLED096

end2:
  ADC_DDR = TXD_MSK;
  #ifdef OLED096
    mgResultScreen();
  #endif
  goto start;
} // end main

//******************************************************************
#ifndef OLED096
// Legacy character-display output helpers (excluded in OLED096 build to preserve Flash)
void UfOutput(uint8_t bcdnum) {
  lcd_line2();
  lcd_fix_string(Uf_str);
  mVOutput(bcdnum >> 4);
  mVOutput(bcdnum & 0x0f);
}

void mVOutput(uint8_t nn) {
  if (nn < 3) {
    DisplayValue(diodes[nn].Voltage,-3,'V',3);
    lcd_space();
  }
}

void RvalOut(uint8_t ii) {
  #if FLASHEND > 0x1fff
    uint16_t rr;
    if ((resis[ii].rx < 200) && (resis[0].lx == 0)) {
      rr = GetESR(resis[ii].ra,resis[ii].rb);
      DisplayValue(rr,-2,LCD_CHAR_OMEGA,3);
    } else {
      DisplayValue(resis[ii].rx,-1,LCD_CHAR_OMEGA,4);
    }
  #else
    DisplayValue(resis[ii].rx,-1,LCD_CHAR_OMEGA,4);
  #endif

  lcd_space();
}
#endif // !OLED096

//******************************************************************
// FUNCTION: ChargePin10ms - Apply a temporary limited gate charge/discharge to a probe.
void ChargePin10ms(uint8_t PinToCharge, uint8_t ChargeDirection) {
  if(ChargeDirection&1) {
    R_PORT |= PinToCharge;
  } else {
    R_PORT &= ~PinToCharge;
  }

  R_DDR |= PinToCharge;
  wait_about10ms();
  R_DDR &= ~PinToCharge;
  R_PORT &= ~PinToCharge;
}

// FUNCTION: EntladePins - Discharge probe-side residual capacitance before active testing.
void EntladePins() {
  uint8_t adc_gnd;
  unsigned int adcmv[3];
  unsigned int clr_cnt;
  uint8_t lop_cnt;

  #define MAX_ENTLADE_ZEIT  (10000/20)

  for(lop_cnt=0;lop_cnt<10;lop_cnt++) {
    adc_gnd = TXD_MSK;
    ADC_DDR = adc_gnd;
    ADC_PORT = TXD_VAL;
    R_PORT = 0;
    R_DDR = (2<<(TP3*2)) | (2<<(TP2*2)) | (2<<(TP1*2));

    adcmv[0] = W5msReadADC(TP1);
    adcmv[1] = ReadADC(TP2);
    adcmv[2] = ReadADC(TP3);

    if ((PartFound == PART_CELL) || ((adcmv[0] < CAP_EMPTY_LEVEL) 
                                  && (adcmv[1] < CAP_EMPTY_LEVEL) 
                                  && (adcmv[2] < CAP_EMPTY_LEVEL))) {
      ADC_DDR = TXD_MSK;
      R_DDR = 0;
      return;
    }

    if (adcmv[0] < 1000) {
      adc_gnd |= (1<<TP1);
    }
    if (adcmv[1] < 1000) {
      adc_gnd |= (1<<TP2);
    }
    if (adcmv[2] < 1000) {
      adc_gnd |= (1<<TP3);
    }
    ADC_DDR = adc_gnd;

    R_DDR = (1<<(TP3*2)) | (1<<(TP2*2)) | (1<<(TP1*2));

    for(clr_cnt=0;clr_cnt<MAX_ENTLADE_ZEIT;clr_cnt++) {
      wdt_reset();
      adcmv[0] = W20msReadADC(TP1);
      adcmv[1] = ReadADC(TP2);
      adcmv[2] = ReadADC(TP3);

      if (adcmv[0] < 1300) {
        ADC_DDR |= (1<<TP1);
      }
      if (adcmv[1] < 1300) {
        ADC_DDR |= (1<<TP2);
      }
      if (adcmv[2] < 1300) {
        ADC_DDR |= (1<<TP3);
      }
      if ((adcmv[0] < (CAP_EMPTY_LEVEL+2)) && (adcmv[1] < (CAP_EMPTY_LEVEL+2)) && (adcmv[2] < (CAP_EMPTY_LEVEL+2))) {
        break;
      }
    }

    if (clr_cnt == MAX_ENTLADE_ZEIT) {
      PartFound = PART_CELL;
    }

    for(adcmv[0]=0;adcmv[0]<clr_cnt;adcmv[0]++) {
      wait1ms();
    }
  }
}

#ifdef AUTO_RH
// FUNCTION: RefVoltage - Compute the capacitance conversion factor from the reference value.
void RefVoltage(void) {
  #define Ref_Tab_Abstand 50
  #define Ref_Tab_Beginn 1000

  unsigned int referenz;
  unsigned int y1, y2;
  uint8_t tabind;
  uint8_t tabres;

  #ifdef AUTO_CAL
    referenz = ref_mv + (int16_t)eeprom_read_word((uint16_t *)(&ref_offset));
  #else
    referenz = ref_mv + REF_C_KORR;
  #endif

  if (referenz >= Ref_Tab_Beginn) {
    referenz -= Ref_Tab_Beginn;
  } else {
    referenz = 0;
  }

  tabind = referenz / Ref_Tab_Abstand;
  tabres = referenz % Ref_Tab_Abstand;
  tabres = Ref_Tab_Abstand-tabres;

  if (tabind > 7) {
    tabind = 7;
  }

  y1 = pgm_read_word(&RHtab[tabind]);
  y2 = pgm_read_word(&RHtab[tabind+1]);
  RHmultip = ((y1 - y2) * tabres + (Ref_Tab_Abstand/2)) / Ref_Tab_Abstand + y2;
}
#endif

#ifdef LCD_CLEAR
void lcd_clear_line(void) {
  unsigned char ll;
  for (ll=0;ll<20;ll++) {
    lcd_space();
  }
}
#endif

// FUNCTION: DisplayValue - Scale an integer engineering measurement and format SI prefixes.
void DisplayValue(unsigned long Value, int8_t Exponent, unsigned char Unit, unsigned char digits)
{
  char OutBuffer[15];
  unsigned int      Limit;
  unsigned char     Prefix;
  uint8_t           Offset;
  uint8_t           Index;
  uint8_t           Length;

  Limit = 100;
  if (digits == 3) Limit = 1000;
  if (digits == 4) Limit = 10000;

  while (Value >= Limit)
  {
    Value += 5;
    Value = Value / 10;
    Exponent++;
  }

  Length = Exponent + 12;
  if ((int8_t)Length <  0) Length = 0;
  if (Length > 18) Length = 18;
  Index = Length / 3;
  Offset = Length % 3;

  if (Offset > 0)
  {
    Index++;
    Offset = 3 - Offset;
  }

  #ifdef NO_NANO
    if (Index == 1)
    {
      Index++;
      Offset += 3;
    }
  #endif

  Prefix = MEM_read_byte((uint8_t *)(&PrefixTab[Index]));

  utoa((unsigned int)Value, OutBuffer, 10);
  Length = strlen(OutBuffer);
  Exponent = Length - Offset;

  if (Exponent <= 0)
  {
    lcd_data('.');

    #ifdef NO_NANO
      while (Exponent < 0)
      {
        lcd_data('0');
        Exponent++;
      }
    #else
      if (Exponent < 0) lcd_data('0');
    #endif
  }

  if (Offset == 0) Exponent = -1;

  Index = 0;
  while (Index < Length)
  {
    lcd_data(OutBuffer[Index]);
    Index++;
    if (Index == Exponent) {
      lcd_data('.');
    }
  }

  if (Prefix != 0) lcd_data(Prefix);
  if (Unit) lcd_data(Unit);
}

#ifndef INHIBIT_SLEEP_MODE
// FUNCTION: sleep_5ms - Implement timed MCU sleep using Timer2 when low-power mode is enabled.
void sleep_5ms(uint16_t pause){
  uint8_t t2_offset;

  #define RESTART_DELAY_US (RESTART_DELAY_TICS/(F_CPU/1000000UL))

  while (pause > 0) {
    #if 3000 > RESTART_DELAY_US
      if (pause > 1) {
        t2_offset = (10000 - RESTART_DELAY_US) / T2_PERIOD;
        pause -= 2;
      } else {
        t2_offset = (5000 - RESTART_DELAY_US) / T2_PERIOD;
        pause = 0;
      }

      OCR2A = TCNT2 + t2_offset;
      TIMSK2 = (0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);

      set_sleep_mode(SLEEP_MODE_PWR_SAVE);
      sleep_mode();
    #else
      wait5ms();
    #endif

    wdt_reset();
  }

  TIMSK2 = (0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);
}
#endif

#if !defined(OLED096) || defined(WITH_SELFTEST)
void PinLayout(char pin1, char pin2, char pin3) {
  #ifndef EBC_STYLE
    lcd_fix_string(N123_str);
    for (ii=0;ii<3;ii++) {
      if (ii == trans.e)  lcd_data(pin1);
      if (ii == trans.b)  lcd_data(pin2);
      if (ii == trans.c)  lcd_data(pin3);
    }
  #else
    #if EBC_STYLE == 321
      lcd_fix_string(N321_str);
      ii = 3;
      while (ii != 0) {
        ii--;
        if (ii == trans.e)  lcd_data(pin1);
        if (ii == trans.b)  lcd_data(pin2);
        if (ii == trans.c)  lcd_data(pin3);
      }
    #else 
      lcd_space();
      lcd_data(pin1);
      lcd_data(pin2);
      lcd_data(pin3);
      lcd_data('=');
      lcd_testpin(trans.e);
      lcd_testpin(trans.b);
      lcd_testpin(trans.c);
    #endif
  #endif
}
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// FUNCTION: AutoCheck - Run optional reference checks and calibration tests when enabled.
void AutoCheck(void) {
#ifdef WITH_SELFTEST

  uint8_t tt;
  uint8_t ww;
  int  adcmv[7];
  uint16_t u680;

  #define MAX_REP 4

  #ifdef AUTO_CAL
    uint8_t cap_found;

    #ifdef AUTOSCALE_ADC
      int8_t udiff;
      int8_t udiff2;
    #endif
  #endif

  ADC_PORT = TXD_VAL;
  ADC_DDR = TXD_MSK;

  #define RequireShortedProbes

  if (AllProbesShorted() != 3) return;

  lcd_clear();
  lcd_fix_string(SELFTEST);

  lcd_line2();
  lcd_fix2_string(R0_str);

  eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[2]), (int8_t)0);
  eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[3]), (int8_t)0);
  eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[1]), (int8_t)0);

  adcmv[0] = GetESR(TP3, TP1);
  adcmv[1] = GetESR(TP3, TP2);
  adcmv[2] = GetESR(TP2, TP1);

  DisplayValue(adcmv[0],-2,' ',3);
  DisplayValue(adcmv[1],-2,' ',3);
  DisplayValue(adcmv[2],-2,LCD_CHAR_OMEGA,3);

  if (adcmv[0] < 60) {
    eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[2]), (int8_t)adcmv[0]);
  }
  if (adcmv[1] < 60) {
    eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[3]), (int8_t)adcmv[1]);
  }
  if (adcmv[2] < 60) {
    eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[1]), (int8_t)adcmv[2]);
  }

  for(tt=0; tt<12; tt++) {
    wait_about500ms();
    if(!(ON_PIN_REG & (1<<RST_PIN))) {
      break;
    }
  }

  #define TEST_COUNT 8
 
  for(tt=1; tt<TEST_COUNT; tt++) {
    for(ww=0; ww<MAX_REP; ww++) {
      lcd_line2();
      lcd_clear_line();
      lcd_line1();
      lcd_clear_line();
      lcd_line1();
      lcd_data('T');
      lcd_string(utoa(tt, outval, 10));
      lcd_space();

      if (tt == 1) {
        Calibrate_UR();
        lcd_fix2_string(URefT);
        DisplayValue(ref_mv,-3,'V',4);
        lcd_line2();
        lcd_fix2_string(RHfakt);
        lcd_string(utoa(RHmultip, outval, 10));
        ADCconfig.Samples = 190;
      }

      if (tt == 2) {
        u680 = ((long)ADCconfig.U_AVCC * (PIN_RM + R_L_VAL) / (PIN_RM + R_L_VAL + R_L_VAL + PIN_RP));
        R_PORT = 1<<(TP1*2);
        R_DDR = (1<<(TP1*2)) | (1<<(TP2*2));
        adcmv[0] = W20msReadADC(TP1);
        adcmv[0] -= u680;
        R_DDR = (1<<(TP1*2)) | (1<<(TP3*2));
        adcmv[1] = W20msReadADC(TP1);
        adcmv[1] -= u680;
        R_PORT = 1<<(TP2*2);
        R_DDR = (1<<(TP2*2)) | (1<<(TP3*2));
        adcmv[2] = W20msReadADC(TP2);
        adcmv[2] -= u680;
        lcd_fix_string(RLRL);
      }

      if (tt == 3) {
        R_PORT = 2<<(TP1*2);
        R_DDR = (2<<(TP1*2)) | (2<<(TP2*2));
        adcmv[0] = W20msReadADC(TP1);
        adcmv[3] = ADCconfig.U_AVCC / 2;
        adcmv[0] -= adcmv[3];
        R_DDR = (2<<(TP1*2)) | (2<<(TP3*2));
        adcmv[1] = W20msReadADC(TP1);
        adcmv[1] -= adcmv[3];
        R_PORT = 2<<(TP2*2);
        R_DDR = (2<<(TP2*2)) | (2<<(TP3*2));
        adcmv[2] = W20msReadADC(TP2);
        adcmv[2] -= adcmv[3];
        lcd_fix_string(RHRH);
      }

      if (tt == 4) {
        lcd_fix_string(RELPROBE);
        if (AllProbesShorted() != 0) ww = MAX_REP-2;
      }

      if (tt == 5) {
        R_PORT = 0;
        R_DDR = 2<<(TP1*2);
        adcmv[0] = W20msReadADC(TP1);

        R_DDR = 2<<(TP2*2);
        adcmv[1] = W20msReadADC(TP2);

        R_DDR = 2<<(TP3*2);
        adcmv[2] = W20msReadADC(TP3);
        lcd_fix_string(RH1L);
      }

      if (tt == 6) {
        R_DDR = 2<<(TP1*2);
        R_PORT = 2<<(TP1*2);
        adcmv[0] = W20msReadADC(TP1) - ADCconfig.U_AVCC;
        R_DDR = 2<<(TP2*2);
        R_PORT = 2<<(TP2*2);
        adcmv[1] = W20msReadADC(TP2) - ADCconfig.U_AVCC;
        R_DDR = 2<<(TP3*2);
        R_PORT = 2<<(TP3*2);
        adcmv[2] = W20msReadADC(TP3) - ADCconfig.U_AVCC;
        lcd_fix_string(RH1H);
      }

      if (tt == 7) {
        u680 = ((long)ADCconfig.U_AVCC * (PIN_RM + R_L_VAL) / (PIN_RM + R_L_VAL + R_H_VAL*100));
        R_PORT = 2<<(TP1*2);
        R_DDR = (2<<(TP1*2)) | (1<<(TP1*2));
        adcmv[0] = W20msReadADC(TP1);
        adcmv[0] -= u680;
        R_PORT = 2<<(TP2*2);
        R_DDR = (2<<(TP2*2)) | (1<<(TP2*2));
        adcmv[1] = W20msReadADC(TP2);
        adcmv[1] -= u680;
        R_PORT = 2<<(TP3*2);
        R_DDR = (2<<(TP3*2)) | (1<<(TP3*2));
        adcmv[2] = W20msReadADC(TP3);
        adcmv[2] -= u680;
        lcd_fix_string(RHRL);
      }

      if (tt > 1) {
        lcd_line2();
        lcd_string(itoa(adcmv[0], outval, 10));
        lcd_space();
        lcd_string(itoa(adcmv[1], outval, 10));
        lcd_space();
        lcd_string(itoa(adcmv[2], outval, 10));
      }

      ADC_DDR =  TXD_MSK;
      ADC_PORT = TXD_VAL;
      R_DDR = 0;
      R_PORT = 0;

      if(!(ON_PIN_REG & (1<<RST_PIN))) {
        break;
      }
      wait_about500ms();

      if(!(ON_PIN_REG & (1<<RST_PIN))) {
        break;
      }
      wait_about500ms();
    }
    wait_about1s();
  }

  lcd_clear();
  lcd_fix_string(RIHI);
  DisplayValue(RRpinPL,-1,LCD_CHAR_OMEGA,3);
  lcd_line2();
  lcd_fix_string(RILO);
  DisplayValue(RRpinMI,-1,LCD_CHAR_OMEGA,3);
  wait_about2s();

  adcmv[3] = 0;
  PartFound = PART_NONE;
  ReadCapacity(TP3, TP1);
  adcmv[5] = (unsigned int) cap.cval_uncorrected.dw;
  ReadCapacity(TP3, TP2);
  adcmv[6] = (unsigned int) cap.cval_uncorrected.dw;
  ReadCapacity(TP2, TP1);
  adcmv[2] = (unsigned int) cap.cval_uncorrected.dw;
  ReadCapacity(TP1, TP3);
  adcmv[1] = (unsigned int) cap.cval_uncorrected.dw;
  ReadCapacity(TP2, TP3);
  adcmv[4] = (unsigned int) cap.cval_uncorrected.dw;
  ReadCapacity(TP1, TP2);
  adcmv[0] = (unsigned int) cap.cval_uncorrected.dw;

  lcd_clear();
  lcd_fix_string(C0_str);
  DisplayValue(adcmv[5],0,' ',3);
  DisplayValue(adcmv[6],0,' ',3);
  DisplayValue(adcmv[2],-12,'F',3);

  #ifdef AUTO_CAL
    for (ww=0;ww<7;ww++) {
      if (adcmv[ww] > 70) goto no_c0save;
    }

    for (ww=0;ww<7;ww++) {
      (void) eeprom_write_byte((uint8_t *)(&c_zero_tab[ww]),adcmv[ww]+(COMP_SLEW1 / (CC0 + CABLE_CAP + COMP_SLEW2)));
    }

    lcd_line2();
    lcd_fix_string(OK_str);

no_c0save:
  #endif

  wait_about2s();
 
  #ifdef AUTO_CAL
    cap_found = 0;
    for (ww=0; ww<64; ww++) {
      lcd_clear();
      lcd_data('1');
      lcd_fix_string(CapZeich);
      lcd_data('3');
      lcd_fix2_string(MinCap_str);

      PartFound = PART_NONE;
      ReadCapacity(TP3, TP1);

      while (cap.cpre < -9) {
        cap.cpre++;
        cap.cval /= 10;
      }

      if ((cap.cpre == -9) && (cap.cval > 95) && (cap.cval < 22000)) {
        cap_found++;
      } else {
        cap_found = 0;
      }

      if (cap_found > 1) {
        (void) eeprom_write_word((uint16_t *)(&ref_offset), load_diff);
        lcd_clear();
        lcd_fix2_string(REF_C_str);
        lcd_string(itoa(load_diff, outval, 10));

        #ifdef AUTOSCALE_ADC
          ADC_PORT =  TXD_VAL;
          ADC_DDR = 1<<TP1 | TXD_MSK;
          R_DDR = 2<<(TP3*2);

          do {
            adcmv[0] = ReadADC(TP3);
          } while (adcmv[0] > 980);

          R_DDR = 0;

          ADCconfig.U_Bandgap = 0;
          adcmv[0] = ReadADC(TP3);
          ADCconfig.U_Bandgap = ADC_internal_reference;
          adcmv[1] = ReadADC(TP3);
          ADCconfig.U_Bandgap = 0;
          adcmv[2] = ReadADC(TP3);
          ADCconfig.U_Bandgap = ADC_internal_reference;

          udiff = (int8_t)(((signed long)(adcmv[0] + adcmv[2] - adcmv[1] - adcmv[1])) * ADC_internal_reference / (2*adcmv[1]))+REF_R_KORR;

          lcd_line2();
          lcd_fix2_string(REF_R_str);

          udiff2 = udiff + (int8_t)eeprom_read_byte((uint8_t *)(&RefDiff));
          (void) eeprom_write_byte((uint8_t *)(&RefDiff), (uint8_t)udiff2);

          lcd_string(itoa(udiff2, outval, 10));
        #endif

        wait_about4s();
        break;
      }

      lcd_line2();
      DisplayValue(cap.cval,cap.cpre,'F',4);
      wait_about200ms();
    }
  #endif

  ADCconfig.Samples = ANZ_MESS;

  lcd_clear();
  lcd_line2();
  lcd_fix2_string(VERSION_str);
  lcd_line1();
  lcd_fix_string(ATE);

  #ifdef FREQUENCY_50HZ
    lcd_fix_string(T50HZ);
    ADC_PORT = TXD_VAL;
    ADC_DDR = 1<<TP1 | TXD_MSK;
    R_DDR = (1<<(TP3*2)) | (1<<(TP2*2));

    for(ww=0;ww<30;ww++) {
      for (ii=0;ii<100;ii++) {
        R_PORT = (1<<(TP2*2));
        wait10ms();
        R_PORT = (1<<(TP3*2));
        wait10ms();
        wdt_reset();
      }

      if (!(ON_PIN_REG & (1<<RST_PIN))) {
        break;
      }
    }
  #endif

  PartFound = PART_NONE;
  wait_about1s();
#endif
} 

#ifdef RequireShortedProbes
uint8_t ShortedProbes(uint8_t Probe1, uint8_t Probe2)
{
  uint8_t           Flag1 = 0;
  unsigned int      U1;
  unsigned int      U2;
  unsigned int      URH;

  R_PORT = pgm_read_byte(&PinRLtab[Probe1]);
  R_DDR = pgm_read_byte(&PinRLtab[Probe1]) | pgm_read_byte(&PinRLtab[Probe2]);

  U1 = ReadADC(Probe1);
  U2 = ReadADC(Probe2);

  URH = ADCconfig.U_AVCC / 2;
  if ((U1 > URH - 20) && (U1 < URH + 20))
  {
    if ((U2 > URH - 20) && (U2 < URH + 20))
    {
      Flag1 = 1;
    }
  }

  R_DDR = 0;
  return Flag1;
}

uint8_t AllProbesShorted(void)
{
  uint8_t Flag2;

  Flag2 = ShortedProbes(TP1, TP2);
  Flag2 += ShortedProbes(TP1, TP3);
  Flag2 += ShortedProbes(TP2, TP3);

  return Flag2;
}
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// FUNCTION: CheckPins - Test a High/Low/Floating probe permutation to identify a part.
void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin) {
  struct {
    unsigned int lp_otr;
    unsigned int hp1;
    unsigned int hp2;
    unsigned int hp3;
    unsigned int lp1;
    unsigned int lp2;
    unsigned int tp1;
    unsigned int tp2;
  } adc;
  uint8_t LoPinRL;
  uint8_t LoPinRH;
  uint8_t TriPinRL;
  uint8_t TriPinRH;
  uint8_t HiPinRL;
  uint8_t HiPinRH;
  uint8_t HiADCp;
  uint8_t LoADCp;
  uint8_t HiADCm;
  uint8_t LoADCm;
  uint8_t PinMSK;
  uint8_t ii;

  #ifdef COMMON_EMITTER
    unsigned int tmp16;
  #endif

  #if FLASHEND > 0x1fff
    int udiff;
  #endif

  #ifdef COMMON_COLLECTOR
    unsigned long c_hfe;
  #endif

  struct resis_t *thisR;
  unsigned long lrx1;
  unsigned long lirx1;
  unsigned long lirx2;

  wdt_reset();

  LoPinRL = pgm_read_byte(&PinRLtab[LowPin]);
  LoPinRH = LoPinRL + LoPinRL;
  TriPinRL = pgm_read_byte(&PinRLtab[TristatePin]);
  TriPinRH = TriPinRL + TriPinRL;
  HiPinRL = pgm_read_byte(&PinRLtab[HighPin]);
  HiPinRH = HiPinRL + HiPinRL;

  HiADCp = pgm_read_byte(&PinADCtab[HighPin]);
  LoADCp = pgm_read_byte(&PinADCtab[LowPin]);
  HiADCm = HiADCp | TXD_MSK;
  HiADCp |= TXD_VAL;
  LoADCm = LoADCp | TXD_MSK;
  LoADCp |= TXD_VAL;

  R_PORT = 0;
  R_DDR = LoPinRL;
  ADC_DDR = HiADCm;
  ADC_PORT = HiADCp;

  ChargePin10ms(TriPinRL,0);
  adc.lp_otr = W5msReadADC(LowPin);
  if (adc.lp_otr >= 977) {
    ChargePin10ms(TriPinRL,1);
    adc.lp_otr = ReadADC(LowPin);
  }

  if(adc.lp_otr > 455) {
    // Test if N-JFET or if self-conducting N-MOSFET
    R_DDR = LoPinRL | TriPinRH;
    adc.lp1 = W20msReadADC(LowPin);
    adc.tp1 = ReadADC(TristatePin);
    R_PORT = TriPinRH;
    adc.lp2 = W20msReadADC(LowPin);

    if (adc.lp2>(adc.lp1+488)) {
      if (PartFound != PART_FET) {
        ADC_PORT = TXD_VAL;
        ADC_DDR = LoADCm;
        R_DDR = TriPinRH | HiPinRL;
        R_PORT = TriPinRH | HiPinRL;
        adc.lp2 = W20msReadADC(TristatePin);

        if (adc.lp2>3911) {
          PartFound = PART_FET;
          PartMode = PART_MODE_N_D_MOS;
        } else {
          PartFound = PART_FET;
          PartMode = PART_MODE_N_JFET;
        }

        trans.uBE[0] = adc.lp1;
        gthvoltage = adc.lp1 - adc.tp1;
        trans.uBE[1] = (unsigned int)(((unsigned long)adc.lp1 * 1000) / RR680MI);
        trans.b = TristatePin;
        trans.c = HighPin;
        trans.e = LowPin;
      }
    }

    ADC_PORT = TXD_VAL;

    // Test if P-JFET or if self-conducting P-MOSFET
    ADC_DDR = LoADCm;
    R_DDR = TriPinRH | HiPinRL;
    R_PORT = TriPinRH | HiPinRL;
    adc.hp1 = W20msReadADC(HighPin);
    adc.tp1 = ReadADC(TristatePin);
    R_PORT = HiPinRL;
    adc.hp2 = W20msReadADC(HighPin);

    if (adc.hp1>(adc.hp2+488)) {
      if (PartFound != PART_FET) {
        ADC_PORT = HiADCp;
        ADC_DDR = HiADCm;
        adc.tp2 = W20msReadADC(TristatePin);

        if (adc.tp2<977) {
          PartFound = PART_FET;
          PartMode = PART_MODE_P_D_MOS;
        } else {
          PartFound = PART_FET;
          PartMode = PART_MODE_P_JFET;
        }

        gthvoltage = adc.tp1 - adc.hp1;
        trans.uBE[1] = (unsigned int)(((unsigned long)(ADCconfig.U_AVCC - adc.hp1) * 1000) / RR680PL);
        trans.b = TristatePin;
        trans.c = LowPin;
        trans.e = HighPin;
      }
    }
  }

  #ifdef COMMON_COLLECTOR
    c_hfe = 0;
    ADC_PORT = TXD_VAL;
    ADC_DDR = LoADCm;
    R_PORT = HiPinRL;
    R_DDR = TriPinRL | HiPinRL;
    adc.hp1 = ADCconfig.U_AVCC - W5msReadADC(HighPin);
    adc.tp1 = ReadADC(TristatePin);

    if (adc.tp1 < 10) {
      R_DDR =  TriPinRH | HiPinRL;
      adc.hp1 = ADCconfig.U_AVCC - W5msReadADC(HighPin);
      adc.tp1 = ReadADC(TristatePin);

      // Require >= 25 mV base drop to prevent small-divisor noise inflation
      if (adc.tp1 >= 25) {
        #ifdef LONG_HFE
          c_hfe = ((unsigned long)adc.hp1 * (unsigned long)(((unsigned long)R_H_VAL * 100) / 
                  (unsigned int)RR680PL)) / (unsigned int)adc.tp1;	
        #else
          c_hfe = ((adc.hp1 / ((RR680PL+500)/1000)) * (R_H_VAL/500)) / (adc.tp1/500);
        #endif
      }
    } else {
      c_hfe = (unsigned long)((adc.hp1 - adc.tp1) / adc.tp1);
    }
  #endif

  R_DDR = LoPinRL;
  R_PORT = 0;
  ADC_DDR = HiADCm;
  ADC_PORT = HiADCp;
  wait_about5ms();
  
  if (adc.lp_otr < 977) {
    // Test PNP
    R_DDR = LoPinRL | TriPinRL;
    adc.lp1 = W5msReadADC(LowPin);

    if (adc.lp1 > 3422) {
      R_DDR = LoPinRL | TriPinRH;
      adc.lp1 = W5msReadADC(LowPin);
      adc.tp2 = ReadADC(TristatePin);

      if ((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) {
        PartReady = 1;
      }

      #ifdef COMMON_EMITTER
        trans.uBE[PartReady] = ReadADC(HighPin) - adc.tp2;

        if(adc.tp2 < 53) {
          adc.tp2 = 53;
        }
        tmp16 = adc.lp1;

        if (tmp16 > adc.lp_otr) {
          tmp16 -= adc.lp_otr;
        }

        #ifdef LONG_HFE
          trans.hfe[PartReady] = ((unsigned int)tmp16 * (unsigned long)(((unsigned long)R_H_VAL * 100) / 
                                 (unsigned int)RR680MI)) / (unsigned int)adc.tp2;	
        #else
          trans.hfe[PartReady] = ((tmp16 / ((RR680MI+500)/1000)) * (R_H_VAL/500)) / (adc.tp2/500);
        #endif
      #endif

      #ifdef COMMON_COLLECTOR
        #ifdef COMMON_EMITTER
          if (c_hfe > trans.hfe[PartReady]) {
        #endif
            trans.hfe[PartReady] = c_hfe;
            trans.uBE[PartReady] = ADCconfig.U_AVCC - adc.hp1 - adc.tp1;
        #ifdef COMMON_EMITTER
          }
        #endif
      #endif

      if (PartFound != PART_THYRISTOR) {
        if (adc.tp2 > 977) {
          PartFound = PART_TRANSISTOR;
          PartMode = PART_MODE_PNP;
        } else {
          if ((adc.lp_otr < 97) && (adc.lp1 > 2000)) {
            PartFound = PART_FET;
            PartMode = PART_MODE_P_E_MOS;
    
            PinMSK = LoADCm & 7;
            ADMUX = TristatePin | (1<<REFS0);
            unsigned long gth_sum = 0;
    
            // 12-iteration threshold averaging scaled to actual AVCC
            for(ii=0;ii<12;ii++) {
              wdt_reset();
              ChargePin10ms(TriPinRL,1);
              R_DDR = LoPinRL | TriPinRH;
              while (!(ADC_PIN&PinMSK));
              R_DDR = LoPinRL;
              ADCSRA |= (1<<ADSC);
              while (ADCSRA&(1<<ADSC));
              gth_sum += (1023 - ADCW);
            }
            gthvoltage = (unsigned int)((gth_sum * ADCconfig.U_AVCC) / (12UL * 1024UL));
          }
        }

        trans.b = TristatePin;
        trans.c = LowPin;
        trans.e = HighPin;
      }
    }

    #ifdef COMMON_COLLECTOR
      c_hfe = 0;
      R_DDR = LoPinRL | TriPinRL;
      R_PORT = TriPinRL;
      adc.lp1 = W5msReadADC(LowPin);
      adc.tp1 = ADCconfig.U_AVCC - ReadADC(TristatePin);
    
      if (adc.tp1 < 10) {
        R_DDR = LoPinRL | TriPinRH;
        R_PORT = TriPinRH;
        adc.lp1 = W5msReadADC(LowPin);
        adc.tp1 = ADCconfig.U_AVCC - ReadADC(TristatePin);
    
        if (adc.tp1 >= 25) {
          #ifdef LONG_HFE
            c_hfe = ((unsigned long)adc.lp1 * (unsigned long)(((unsigned long)R_H_VAL * 100) / 
                    (unsigned int)RR680MI)) / (unsigned int)adc.tp1;	
          #else
            c_hfe = ((adc.lp1 / ((RR680MI+500)/1000)) * (R_H_VAL/500)) / (adc.tp2/500);
          #endif
        }
      } else {
        c_hfe = (adc.lp1 - adc.tp1) / adc.tp1;
      }
    #endif

    // Tristate (can be Base) to VCC, Test if NPN
    ADC_DDR = LoADCm;
    ADC_PORT = TXD_VAL;
    R_DDR = TriPinRL | HiPinRL;
    R_PORT = TriPinRL | HiPinRL;
    adc.hp1 = W5msReadADC(HighPin);
    
    if (adc.hp1 < 1600) {
      if (PartReady==1) {
        goto widmes;
      }
    
      // Test for a thyristor / TRIAC
      ChargePin10ms(TriPinRL,0);
      adc.hp3 = W5msReadADC(HighPin);
    
      R_PORT = 0;
      wait_about5ms();
      R_PORT = HiPinRL;
      adc.hp2 = W5msReadADC(HighPin);
    
      if ((adc.hp3 < 1600) && (adc.hp2 > 4400)) {
        PartFound = PART_THYRISTOR;
    
        // Test if Triac (bidirectional conduction)
        R_DDR = 0;
        R_PORT = 0;
        ADC_PORT = LoADCp;
        wait_about5ms();
    
        R_DDR = HiPinRL;
        if(W5msReadADC(HighPin) > 244) {
          goto savenresult;
        }
    
        R_DDR = HiPinRL | TriPinRL;
        if(W5msReadADC(TristatePin) < 977) {
          goto savenresult;
        }
    
        if(ReadADC(HighPin) < 733) {
          goto savenresult;
        }
    
        R_DDR = HiPinRL;
        if(W5msReadADC(HighPin) < 733) {
          goto savenresult;
        }
    
        R_PORT = HiPinRL;
        wait_about5ms();
        R_PORT = 0;
        if(W5msReadADC(HighPin) > 244) {
          goto savenresult;
        }
    
        PartFound = PART_TRIAC;
        PartReady = 1;
        goto savenresult;
      }
    
      // Test if NPN Transistor or MOSFET
      R_DDR = HiPinRL | TriPinRH;
      R_PORT = HiPinRL | TriPinRH;
      wait_about50ms();
      adc.hp2 = ADCconfig.U_AVCC - ReadADC(HighPin);
      adc.tp2 = ADCconfig.U_AVCC - ReadADC(TristatePin);
    
      if((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) {
        PartReady = 1;
      }
    
      #ifdef COMMON_EMITTER
        trans.uBE[PartReady] = ADCconfig.U_AVCC - adc.tp2 - ReadADC(LowPin);
    
        if (adc.tp2 < 53) {
          adc.tp2 = 53;
        }
    
        tmp16 = adc.hp2;
        if (tmp16 > adc.lp_otr) {
          tmp16 -= adc.lp_otr;
        }
    
        #ifdef LONG_HFE
          trans.hfe[PartReady] = ((unsigned int)tmp16 * (unsigned long)(((unsigned long)R_H_VAL * 100) / 
                                 (unsigned int)RR680PL)) / (unsigned int)adc.tp2;	
        #else
          trans.hfe[PartReady] = ((tmp16 / ((RR680PL+500)/1000)) * (R_H_VAL/500)) / (adc.tp2/500);
        #endif
      #endif
    
      #ifdef COMMON_COLLECTOR
        #ifdef COMMON_EMITTER
          if (c_hfe >  trans.hfe[PartReady]) {
        #endif
            trans.hfe[PartReady] = c_hfe;
            trans.uBE[PartReady] = ADCconfig.U_AVCC - adc.lp1 - adc.tp1;
        #ifdef COMMON_EMITTER
          }
        #endif
      #endif
    
      if(adc.tp2 > 2557) {
        PartFound = PART_TRANSISTOR;
        PartMode = PART_MODE_NPN;
      } else {
        if((adc.lp_otr < 97) && (adc.hp2 > 3400)) {
          PartFound = PART_FET;
          PartMode = PART_MODE_N_E_MOS;
    
          PinMSK = HiADCm & 7;
          ADMUX = TristatePin | (1<<REFS0);
          unsigned long gth_sum = 0;
    
          // 12-iteration threshold averaging scaled to actual AVCC
          for(ii=0;ii<12;ii++) {
            wdt_reset();
            ChargePin10ms(TriPinRL,0);
            R_DDR = HiPinRL | TriPinRH;
            R_PORT = HiPinRL | TriPinRH;
            while ((ADC_PIN&PinMSK));
            R_DDR = HiPinRL;
            ADCSRA |= (1<<ADSC);
            while (ADCSRA&(1<<ADSC));
            gth_sum += ADCW;
          }
    
          gthvoltage = (unsigned int)((gth_sum * ADCconfig.U_AVCC) / (12UL * 1024UL));
        }
      }

savenresult:
      trans.b = TristatePin;
      trans.c = HighPin;
      trans.e = LowPin;
    }

    ADC_DDR = TXD_MSK;
    ADC_PORT = TXD_VAL;

    goto widmes;
  }

  // Test if Diode / LED
  ADC_PORT = TXD_VAL;

  for (ii=0;ii<200;ii++) {
    ADC_DDR = LoADCm | HiADCm;
    wait_about5ms();
    ADC_DDR = LoADCm;
    adc.hp1 = ReadADC(HighPin);
    if (adc.hp1 < (150/8)) break;
  }

  R_DDR = HiPinRH;
  R_PORT = HiPinRH;
  ChargePin10ms(TriPinRL,1);
  adc.hp2 = W5msReadADC(HighPin);
  ChargePin10ms(TriPinRL,0);
  adc.hp3 = W5msReadADC(HighPin);
  
  R_DDR = HiPinRL;
  R_PORT = HiPinRL;
  adc.hp1 = W5msReadADC(HighPin) - ReadADC(LowPin);
  ChargePin10ms(TriPinRL,1);
  adc.lp_otr = W5msReadADC(HighPin) - ReadADC(LowPin);
  
  R_DDR = HiPinRH;
  R_PORT = HiPinRH;

  if(adc.lp_otr > adc.hp1) {
    adc.hp1 = adc.lp_otr;
    adc.hp3 = adc.hp2;
  } else {
    ChargePin10ms(TriPinRL,0);
  }

  adc.hp2 = W5msReadADC(HighPin);

  if((adc.hp1 > 150) && (adc.hp1 < 4640) && (adc.hp2 < adc.hp1) && (adc.hp1 > (adc.hp3+(adc.hp3/8))) && (adc.hp3*16 > adc.hp1)) {
    if((PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) {
      PartFound = PART_DIODE;
    }

    diodes[NumOfDiodes].Anode = HighPin;
    diodes[NumOfDiodes].Cathode = LowPin;
    diodes[NumOfDiodes].Voltage = adc.hp1;
    NumOfDiodes++;
  }

widmes:
  if (NumOfDiodes > 0) goto clean_ports;
  wdt_reset();

  #if U_SCALE != 1
    ADCconfig.U_AVCC *= U_SCALE;
    ADCconfig.U_Bandgap *= U_SCALE;
  #endif

  #if R_ANZ_MESS != ANZ_MESS
    ADCconfig.Samples = R_ANZ_MESS;
  #endif

  #define MAX_REPEAT (700 / (5 + R_ANZ_MESS/8))

  ADC_PORT = TXD_VAL;
  ADC_DDR = LoADCm;
  R_DDR = HiPinRL;
  R_PORT = HiPinRL;	

  #if FLASHEND > 0x1fff
    adc.hp2 = 0;

    for (ii=1;ii<MAX_REPEAT;ii++) {
      adc.tp1 = W5msReadADC(LowPin);
      adc.hp1 = ReadADC(HighPin);
      udiff = adc.hp1 - adc.hp2;
      if (udiff < 0) udiff = -udiff;
      if (udiff < 3) break;
      adc.hp2 = adc.hp1;
      wdt_reset();
    }

    if (ii == MAX_REPEAT) goto testend;
  #else
    adc.tp1 = W5msReadADC(LowPin);
    adc.hp1 = ReadADC(HighPin);
  #endif

  if (adc.tp1 > adc.hp1) {
    adc.tp1 = adc.hp1;
  }

  R_PORT = 0;
  R_DDR = HiPinRH;
  adc.hp2 = W5msReadADC(HighPin);

  if (adc.hp2 > (20*U_SCALE)) {
    goto testend;
  }

  R_PORT = HiPinRH;
  adc.hp2 = W5msReadADC(HighPin);

  ADC_DDR = HiADCm;
  ADC_PORT = HiADCp;
  R_PORT = 0;
  R_DDR = LoPinRL;

  #if FLASHEND > 0x1fff
    adc.lp2 = 0;

    for (ii=1;ii<MAX_REPEAT;ii++) {
      adc.tp2 = W5msReadADC(HighPin);
      adc.lp1 = ReadADC(LowPin);
      udiff = adc.lp1 - adc.lp2;
      if (udiff < 0) udiff = -udiff;
      if (udiff < 3) break;
      adc.lp2 = adc.lp1;
      wdt_reset();
    }

    if (ii == MAX_REPEAT) goto testend;
  #else
    adc.tp2 = W5msReadADC(HighPin);
    adc.lp1 = ReadADC(LowPin);
  #endif

  if (adc.tp2 < adc.lp1) {
    adc.tp2 = adc.lp1;
  }

  R_DDR = LoPinRH;
  adc.lp2 = W5msReadADC(LowPin);

  if((adc.hp1 < (4400*U_SCALE)) && (adc.hp2 > (97*U_SCALE))) {
    goto testend; 
  }

  if (adc.hp2 < (4972*U_SCALE)) { 
    if (adc.lp1 < (169*U_SCALE)) {
      ii = 'H';

      if (adc.lp2 < (38*U_SCALE)) {
        goto testend;
      }

      lirx1 = (unsigned long)((unsigned int)R_H_VAL) * (unsigned long)adc.hp2 / (ADCconfig.U_AVCC - adc.hp2);
      lirx2 = (unsigned long)((unsigned int)R_H_VAL) * (unsigned long)(ADCconfig.U_AVCC - adc.lp2) / adc.lp2;

      #define U_INT_LIMIT (990*U_SCALE)

      #ifdef __AVR_ATmega8__
        #define FAKT_LOW 2
      #else
        #define FAKT_LOW 4
      #endif

      #ifdef AUTOSCALE_ADC
        if (adc.hp2 < U_INT_LIMIT) {
          lrx1 = (lirx1*FAKT_LOW + lirx2) / (FAKT_LOW+1);
        } else if (adc.lp2 < U_INT_LIMIT){
          lrx1 = (lirx2*FAKT_LOW + lirx1) / (FAKT_LOW+1);
        } else 
      #endif
        {
          lrx1 = (lirx1 + lirx2) / 2;
        }

      lrx1 *= 100;
      lrx1 += RH_OFFSET;

    } else {
      ii = 'L';

      if (adc.tp1 > adc.hp1) {
        adc.hp1 = adc.tp1;
      }

      lirx1 =(unsigned long)RR680PL * (unsigned long)(adc.hp1 - adc.tp1) / (ADCconfig.U_AVCC - adc.hp1);

      if (adc.tp2 < adc.lp1) {
        adc.lp1 = adc.tp2;
      }

      lirx2 =(unsigned long)RR680MI * (unsigned long)(adc.tp2 -adc.lp1) / adc.lp1;

      #ifdef AUTOSCALE_ADC
        if (adc.hp1 < U_INT_LIMIT) {
          lrx1 = (lirx1*FAKT_LOW + lirx2) / (FAKT_LOW+1);
        } else if (adc.lp1 < U_INT_LIMIT) {
          lrx1 = (lirx2*FAKT_LOW + lirx1) / (FAKT_LOW+1);
        } else
      #endif
        {
          lrx1 = (lirx1 + lirx2) / 2;
        }

      // Subtract residual probe/trace zero offset (0.1 Ohm units) on low-ohm range
      uint8_t rzero = pgm_read_byte(&EE_ESR_ZEROtab[LowPin + HighPin]) / 10;
      if (lrx1 > rzero) {
        lrx1 -= rzero;
      } else {
        lrx1 = 0;
      }
    }

    if((PartFound == PART_DIODE) || (PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) {
      for (ii=0; ii<ResistorsFound; ii++) {
        thisR = &resis[ii];
        if (thisR->rt != TristatePin) continue;

        unsigned long rdiff = (labs((long)lrx1 - (long)thisR->rx) * 10) / (lrx1 + thisR->rx + 100);

        if (rdiff > 0) {
          goto testend;
        }

        // Average forward and reverse polarity readings to cancel thermal/pin asymmetry
        thisR->rx = (thisR->rx + lrx1 + 1) / 2;
        PartFound = PART_RESISTOR;
        goto testend;
      }

      thisR = &resis[ResistorsFound];
      thisR->rx = lrx1;

      #if FLASHEND > 0x1fff
        thisR->lx = 0;
      #endif

      thisR->ra = LowPin;
      thisR->rb = HighPin;
      thisR->rt = TristatePin;
      ResistorsFound++;
    }
  }

testend:

  #if U_SCALE != 1
    ADCconfig.U_AVCC /= U_SCALE;
    ADCconfig.U_Bandgap /= U_SCALE;
  #endif

  #if R_ANZ_MESS != ANZ_MESS
    ADCconfig.Samples = ANZ_MESS;
  #endif

clean_ports:

  ADC_DDR = TXD_MSK;
  ADC_PORT = TXD_VAL;
  R_DDR = 0;
  R_PORT = 0;
}  // end CheckPins()

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

#ifndef OLED096
// FUNCTION: GetIr - Estimate the diode reverse-leakage current (legacy LCD mode only).
void GetIr(uint8_t hipin, uint8_t lopin) {
  unsigned int u_res;
  unsigned int ir_nano;
  uint8_t LoPinR_L;
  uint8_t HiADC;

  HiADC = pgm_read_byte(&PinADCtab[hipin]);
  ADC_PORT = HiADC | TXD_VAL;
  ADC_DDR = HiADC | TXD_MSK;
  LoPinR_L = pgm_read_byte(&PinRLtab[lopin]);
  R_PORT = 0;
  R_DDR = LoPinR_L + LoPinR_L;

  u_res = W5msReadADC(lopin);
  if (u_res == 0) return;

  #if defined(NOK5110)
    lcd_line4();
  #endif

  lcd_fix_string(Ir_str);

  ir_nano = (unsigned long)(u_res * 100000UL) / R_H_VAL;
  DisplayValue(ir_nano,-9,'A',2);

  ADC_DDR = TXD_MSK;
  ADC_PORT = TXD_VAL;
  R_DDR = 0;
}
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

#ifdef INHIBIT_SLEEP_MODE
  #define StartADCwait() ADCSRA = StartADCmsk; \
  while (ADCSRA & (1 << ADSC))
#else
  #define StartADCwait() ADCSRA = (1<<ADEN) | (1<<ADIF) | (1<<ADIE) | AUTO_CLOCK_DIV; \
  set_sleep_mode(SLEEP_MODE_ADC);\
  sleep_mode();
#endif

// FUNCTION: ReadADC - Take multi-sample ADC reading with rounded 10-bit scaling.
unsigned int ReadADC (uint8_t Probe) {
  unsigned int U;
  uint8_t Samples;
  unsigned long Value;
  Probe |= (1 << REFS0);

  #ifdef AUTOSCALE_ADC
sample:
  #endif

  ADMUX = Probe;

  #ifdef AUTOSCALE_ADC
    if ((Probe & (1 << REFS1)) != 0) {
      #ifdef NO_AREF_CAP
        wait250us(); // Extended settling time for internal 1.1V bandgap stability
      #else
        wait_about10ms();
      #endif
    }
  #endif

  StartADCwait();

  Value = 0UL;
  Samples = 0;

  while (Samples < ADCconfig.Samples) {
    StartADCwait();
    Value += ADCW;

    #ifdef AUTOSCALE_ADC
      if ((Samples == 4) && (ADCconfig.U_Bandgap > 255) && ((uint16_t)Value < 1024) && !(Probe & (1 << REFS1))) {
        Probe |= (1 << REFS1);

        #if PROCESSOR_TYP == 1280
          Probe &= ~(1 << REFS0);
        #endif

        goto sample;
      }
    #endif

    Samples++;
  }

  #ifdef AUTOSCALE_ADC
    if (Probe & (1 << REFS1)) U = ADCconfig.U_Bandgap;
    else U = ADCconfig.U_AVCC;
  #else                                 
    U = ADCconfig.U_AVCC;
  #endif

  // Rounded 10-bit ADC scaling (/1024 instead of truncated /1023)
  Value *= U;
  Value += (ADCconfig.Samples * 512UL);
  Value /= 1024;
  Value /= ADCconfig.Samples;
  U = (unsigned int)Value;
  return U;
}

unsigned int W5msReadADC (uint8_t Probe) {
  wait_about5ms();
  return (ReadADC(Probe));
}

unsigned int W10msReadADC (uint8_t Probe) {
  wait_about10ms();
  return (ReadADC(Probe));
}

unsigned int W20msReadADC (uint8_t Probe) {
  wait_about20ms();
  return (ReadADC(Probe));
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// FUNCTION: ReadCapacity - Estimate capacitance from timed charging on the probe network.
void ReadCapacity(uint8_t HighPin, uint8_t LowPin) {
  unsigned int tmpint;
  unsigned int adcv[4];

  #ifdef INHIBIT_SLEEP_MODE
    unsigned int ovcnt16;
  #endif

  uint8_t HiPinR_L, HiPinR_H;
  uint8_t LoADC;
  uint8_t ii;

  #if FLASHEND > 0x1fff
    unsigned int vloss;
  #endif

  #ifdef AUTO_CAL
    pin_combination = (HighPin * 3) + LowPin - 1;
  #endif

  LoADC = pgm_read_byte(&PinADCtab[LowPin]) | TXD_MSK;
  HiPinR_L = pgm_read_byte(&PinRLtab[HighPin]);
  HiPinR_H = HiPinR_L + HiPinR_L;

  if(PartFound == PART_RESISTOR) {
    return;
  }

  for (ii=0;ii<NumOfDiodes;ii++) {
    if ((diodes[ii].Cathode == LowPin) && (diodes[ii].Anode == HighPin) && (diodes[ii].Voltage < 1350)) {
      return;
    }
  }
  
  #if FLASHEND > 0x1fff
    cap.esr = 0;
    vloss = 0;
  #endif

  cap.cval = 0;
  cap.cpre = -12;
  EntladePins();

  ADC_PORT = TXD_VAL;
  R_PORT = 0;
  ADC_DDR = LoADC;
  R_DDR = HiPinR_L;
  adcv[0] = ReadADC(HighPin);

  adcv[2] = adcv[0];

  for (ovcnt16=0; ovcnt16<500; ovcnt16++) {
    R_PORT = HiPinR_L;
    R_DDR = HiPinR_L;
    wait10ms();

    R_DDR = 0;
    R_PORT = 0;
    wait500us();

    wdt_reset();

    adcv[2] = ReadADC(HighPin);

    if (adcv[2] > adcv[0]) {
      adcv[2] -= adcv[0];
    } else {
      adcv[2] = 0;
    }

    if ((ovcnt16 == 126) && (adcv[2] < 75)) {
      break;
    }

    if (adcv[2] > 300) {
      break;
    }
  }

  if (adcv[2] < 301) {
    goto keinC;
  }

  if ((ovcnt16 == 0 ) && (adcv[2] > 1300)) {
    goto messe_mit_rh;
  }

  #ifdef NO_CAP_HOLD_TIME
    ChargePin10ms(HiPinR_H,0);
    adcv[3] = ReadADC(HighPin) - adcv[0];

    if (adcv[3] > adcv[0]) {
      adcv[3] -= adcv[0];
    } else {
      adcv[3] = 0;
    }

    if ((adcv[3] + adcv[3]) < adcv[2]) {
      if (ovcnt16 == 0 ) {
        goto messe_mit_rh;
      }
      goto keinC;
    }

    cap.cval_uncorrected.dw = ovcnt16 + 1;
    cap.cval_uncorrected.dw *= getRLmultip(adcv[2]);

  #else
    adcv[3] = adcv[2];

    for (tmpint=0; tmpint<=ovcnt16; tmpint++) {
      wait5ms();
      adcv[3] = ReadADC(HighPin);
      wdt_reset();
    }

    if (adcv[3] > adcv[0]) {
      adcv[3] -= adcv[0];
    } else {
      adcv[3] = 0;
    }

    if (adcv[2] > adcv[3]) {
      adcv[3] = adcv[2] - adcv[3];
    } else {
      adcv[3] = 0;
    }

    #if FLASHEND > 0x1fff
      if (adcv[3] > 0) {
        vloss = (unsigned long)(adcv[3] * 1000UL) / adcv[2];
      }
    #endif

    if (adcv[3] > 100) {
      if (ovcnt16 == 0 ) {
        goto messe_mit_rh;
      }
      goto keinC;
    }

    cap.cval_uncorrected.dw = ovcnt16 + 1;
    // Half-drop compensation prevents overestimating low-leakage electrolytic capacitors
    cap.cval_uncorrected.dw *= getRLmultip(adcv[2] + (adcv[3] >> 1));
  #endif

  cap.cval = cap.cval_uncorrected.dw;
  cap.cpre = -9;
  Scale_C_with_vcc();

  cap.cval *= (1000 - C_H_KORR);
  cap.cval /= 100;

  goto checkDiodes;

//==================================================================================
messe_mit_rh:
  EntladePins();

  R_PORT = 0;
  ADC_DDR = (1<<TP1) | (1<<TP2) | (1<<TP3) | TXD_MSK;
  ADC_PORT = TXD_VAL;
  R_DDR = HiPinR_H;

  ADC_COMP_CONTROL = (1<<ACME);
  ACSR =  (1<<ACBG) | (1<<ACI)  | (1<<ACIC);
  ADMUX = (1<<REFS0) | HighPin;
  ADCSRA = (1<<ADIF) | AUTO_CLOCK_DIV;
  wait250us();

  ovcnt16 = 0;
  TCCR1A = 0;
  TCNT1 = 0;
  TI1_INT_FLAGS = (1<<ICF1) | (1<<OCF1B) | (1<<OCF1A) | (1<<TOV1);

  #ifndef INHIBIT_SLEEP_MODE
    TIMSK1 = (1<<TOIE1) | (1<<ICIE1);
    unfinished = 1;
  #endif

  R_PORT = HiPinR_H;

  if(PartFound == PART_FET) {
    TCCR1B = (1<<CS10);
    ADC_DDR = (((1<<TP1) | (1<<TP2) | (1<<TP3) | TXD_MSK) & ~(1<<HighPin));
  } else {
    TCCR1B =  (1<<CS10);
    ADC_DDR = LoADC;
  }

  #ifdef INHIBIT_SLEEP_MODE
    while(1) {
      ii = TI1_INT_FLAGS;

      if (ii & (1<<ICF1))  {
        break;
      }

      if((ii & (1<<TOV1))) {
        TI1_INT_FLAGS = (1<<TOV1);
        wdt_reset();
        ovcnt16++;

        if(ovcnt16 == (F_CPU/5000)) {
          break;
        }
      }
    }

    TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<CS10);
    TI1_INT_FLAGS = (1<<ICF1);
    tmpint = ICR1;

    if((TCNT1 > tmpint) && (ii & (1<<TOV1))) {
      TI1_INT_FLAGS = (1<<TOV1);
      ovcnt16++;
    }

  #else
    while(unfinished) {
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_mode();
      wdt_reset();

      if(ovcnt16 == (F_CPU/5000)) {
        break;
      }
    }

    TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<CS10);
    tmpint = ICR1;
    TIMSK1 = (0<<TOIE1) | (0<<ICIE1);

    if (TCNT1 < tmpint) {
      ovcnt16--;
    }
  #endif

  ADCSRA = (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV;
  R_DDR = 0;
  R_PORT = 0;
  adcv[2] = ReadADC(HighPin);
  load_diff = adcv[2] + REF_C_KORR - ref_mv;

  if (ovcnt16 >= (F_CPU/10000)) {
    goto keinC;
  }

  cap.cval_uncorrected.w[1] = ovcnt16;
  cap.cval_uncorrected.w[0] = tmpint;

  cap.cpre = -12;
  if (ovcnt16 > 65) {
    cap.cval_uncorrected.dw /= 100;
    cap.cpre += 2;
  }

  cap.cval_uncorrected.dw *= RHmultip;
  cap.cval_uncorrected.dw /= (F_CPU / 10000);
  cap.cval = cap.cval_uncorrected.dw;
  Scale_C_with_vcc();

  if (cap.cpre == -12) {
    #if COMP_SLEW1 > COMP_SLEW2
      if (cap.cval < COMP_SLEW1) {
        cap.cval += (COMP_SLEW1 / (cap.cval+COMP_SLEW2 ));
      }
    #endif

    #ifdef AUTO_CAL
      tmpint = eeprom_read_byte(&c_zero_tab[pin_combination]);

      if (cap.cval > tmpint) {
        cap.cval -= tmpint;
      } else {
        cap.cval = 0;
      }

    #else
      // Subtract parasitic offset (TP2 has higher stray capacitance to adjacent pins)
      uint16_t c_zero = C_NULL + ((HighPin == TP2 || LowPin == TP2) ? TP2_CAP_OFFSET : 0);

      if (cap.cval > c_zero) {
        cap.cval -= c_zero;
      } else {
        cap.cval = 0;
      }
    #endif
  }

  R_DDR = HiPinR_L;

  #if F_CPU < 2000001
    if(cap.cval < 50)
  #else 
    if(cap.cval < 25)
  #endif 
    {
      goto keinC;
    }

checkDiodes:

  if((NumOfDiodes > 0) && (PartFound != PART_FET)) {
    // Diode forward path present
  } else {
    PartFound = PART_CAPACITOR;

    if ((cap.cpre > cap.cpre_max) || ((cap.cpre == cap.cpre_max) && (cap.cval > cap.cval_max))) {
      cap.cval_max = cap.cval;
      cap.cpre_max = cap.cpre;

      #if FLASHEND > 0x1fff
        cap.v_loss = vloss;
      #endif

      cap.ca = LowPin;
      cap.cb = HighPin;
    }
  }

keinC:
  ADC_DDR =  TXD_MSK;
  ADC_PORT = TXD_VAL;
  R_DDR = 0;
  R_PORT = 0;

  return;
}  // end ReadCapacity()

unsigned int getRLmultip(unsigned int cvolt) {
  #define RL_Tab_Abstand 25
  #define RL_Tab_Beginn 300
  #define RL_Tab_Length 1100

  unsigned int uvolt;
  unsigned int y1, y2;
  uint8_t tabind;
  uint8_t tabres;

  if (cvolt >= RL_Tab_Beginn) {
    uvolt = cvolt - RL_Tab_Beginn;
  } else {
    uvolt = 0;
  }

  tabind = uvolt / RL_Tab_Abstand;
  tabres = uvolt % RL_Tab_Abstand;
  tabres = RL_Tab_Abstand - tabres;

  if (tabind > (RL_Tab_Length/RL_Tab_Abstand)) {
    tabind = (RL_Tab_Length/RL_Tab_Abstand);
  }

  y1 = MEM_read_word(&RLtab[tabind]);
  y2 = MEM_read_word(&RLtab[tabind+1]);
  return ( ((y1 - y2) * tabres + (RL_Tab_Abstand/2)) / RL_Tab_Abstand + y2);
}

void Scale_C_with_vcc(void) {
  while (cap.cval > 100000) {
    cap.cval /= 10;
    cap.cpre ++;
  }

  cap.cval *= ADCconfig.U_AVCC;
  cap.cval /= U_VCC;
}

#ifndef INHIBIT_SLEEP_MODE
ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
  ovcnt16++;
}

ISR(TIMER1_CAPT_vect, ISR_BLOCK)
{
  unfinished = 0;
}
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// FUNCTION: ReadInductance - Estimate inductance for the compatible resistor/inductor mode.
void ReadInductance(void) {
#if FLASHEND > 0x1fff

  unsigned int tmpint;
  unsigned int umax;
  unsigned int total_r;
  unsigned int mess_r;
  unsigned long inductance[4];

  union t_combi{
  unsigned long dw;
  uint16_t w[2];
  } timeconstant;

  uint16_t per_ref1,per_ref2;
  uint8_t LoPinR_L;
  uint8_t HiADC;
  uint8_t ii;
  uint8_t count;

  #define found 0

  uint8_t cnt_diff;
  uint8_t LowPin;
  uint8_t HighPin;
  int8_t ukorr;
  uint8_t nr_pol1;
  uint8_t nr_pol2;

  if(PartFound != PART_RESISTOR) {
    return;
  }

  if (ResistorsFound != 1) {
    return;
  }

  if (resis[found].rx > 21000) return;

  for (count=0; count<4; count++) {
    if (count < 2) {
      LowPin = resis[found].ra;
      HighPin = resis[found].rb;
    } else {
      LowPin = resis[found].rb;
      HighPin = resis[found].ra;
    }

    HiADC = pgm_read_byte(&PinADCtab[HighPin]);
    LoPinR_L = pgm_read_byte(&PinRLtab[LowPin]);

    R_PORT = 0;
    ADC_PORT = TXD_VAL;

    if ((resis[found].rx < 240) && ((count & 0x01) == 0)) {
      mess_r = RR680MI - R_L_VAL;
      ADC_DDR = HiADC | (1<<LowPin) | TXD_MSK;
    } else {
      R_DDR = LoPinR_L;
      ADC_DDR = HiADC | TXD_MSK;
      mess_r = RR680MI;
    }

    for (ii=0;ii<20;ii++) {
      umax = W10msReadADC(LowPin);
      total_r =  ReadADC(HighPin);
      if ((umax < 2) && (total_r < 2)) break;
    }

    ADC_COMP_CONTROL = (1<<ACME);
    ACSR =  (1<<ACBG) | (1<<ACI)  | (1<<ACIC);
    ADMUX = (1<<REFS0) | LowPin;
    ADCSRA = (1<<ADIF) | AUTO_CLOCK_DIV;
   
    timeconstant.w[1] = 0;
    TCCR1A = 0;
    TCNT1 = 0;
    TI1_INT_FLAGS = (1<<ICF1) | (1<<OCF1B) | (1<<OCF1A) | (1<<TOV1);
    HiADC |= TXD_VAL;
    wait250us();

    if ((count & 0x01) == 0 ) {
      TCCR1B =  (1<<ICNC1) | (0<<ICES1) | (1<<CS10);
      ADC_PORT = HiADC;
    } else {
      ADC_PORT = HiADC;

      #if F_CPU >= 8000000UL
        wait3us();
      #else
        wdt_reset();
        wdt_reset();
      #endif

      TI1_INT_FLAGS = (1<<ICF1);
      TCCR1B =  (1<<ICNC1) | (0<<ICES1) | (1<<CS10);
    }
      
    while(1) {
      ii = TI1_INT_FLAGS;

      if (ii & (1<<ICF1))  {
        break;
      }

      if((ii & (1<<TOV1))) {
        TI1_INT_FLAGS = (1<<TOV1);
        wdt_reset();
        timeconstant.w[1]++;

        if(timeconstant.w[1] == (F_CPU/100000UL)) {
          break;
        }
      }
    }

    TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<CS10);
    TI1_INT_FLAGS = (1<<ICF1);
    timeconstant.w[0] = ICR1;

    if((TCNT1 > timeconstant.w[0]) && (ii & (1<<TOV1))) {
      TI1_INT_FLAGS = (1<<TOV1);
      timeconstant.w[1]++;
    }

    ADC_PORT = TXD_VAL;
    ADCSRA = (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV;

    for (ii=0;ii<20;ii++) {
      umax = W10msReadADC(LowPin);
      total_r =  ReadADC(HighPin);

      if ((umax < 2) && (total_r < 2)) break;
    }

    #define CNT_ZERO_42 6
    #define CNT_ZERO_720 7

    total_r = (mess_r + resis[found].rx + RRpinMI);

    cnt_diff = total_r / ((14000UL * 8) / (F_CPU/1000000UL));

    #ifdef AUTO_CAL
      tmpint = (ref_mv + (int16_t)eeprom_read_word((uint16_t *)(&ref_offset))) ;
    #else
      tmpint = (ref_mv + REF_C_KORR);
    #endif

    if (mess_r < R_L_VAL) {
      cnt_diff = CNT_ZERO_42;

      if (timeconstant.dw < 225) {
        ukorr = (timeconstant.w[0] / 5) - 20;
      } else {
        ukorr = 25;
      }

      tmpint -= (((REF_L_KORR * 10) / 10) + ukorr);
    } else {
      cnt_diff += CNT_ZERO_720;
      tmpint += REF_L_KORR;
    }

    if (timeconstant.dw > cnt_diff) timeconstant.dw -= cnt_diff;
    else timeconstant.dw = 0;
       
    if ((count&0x01) == 1) {
      timeconstant.dw += (3 * (F_CPU/1000000UL))+10;
    }

    if (timeconstant.w[1] >= (F_CPU/100000UL)) timeconstant.dw = 0;

    if (timeconstant.dw > 10) {
      timeconstant.dw -= 1;
    }

    umax = ((unsigned long)mess_r * (unsigned long)ADCconfig.U_AVCC) / total_r;
    per_ref1 = ((unsigned long)tmpint * 1000) / umax;
    per_ref2 = get_log(per_ref1);

    per_ref1 = ((per_ref2 * (F_CPU/1000000UL)) + 5) / 10;
    inductance[count] = (timeconstant.dw * total_r ) / per_ref1;

    if (((count&0x01) == 0) && (timeconstant.dw > ((F_CPU/1000000UL)+3))) {
      inductance[count+1] = inductance[count];
      count++;
    }

    wdt_reset();
  }

  ADC_PORT = TXD_VAL;
  wait_about20ms();

  nr_pol1 = 0;
  if (inductance[1] > inductance[0]) { nr_pol1 = 1; } 

  nr_pol2 = 2;
  if (inductance[3] > inductance[2]) { nr_pol2 = 3; } 

  if (inductance[nr_pol2] < inductance[nr_pol1]) nr_pol1 = nr_pol2;

  resis[found].lx = inductance[nr_pol1];
  resis[found].lpre = -5;

  if (((nr_pol1 & 1) == 1) || (resis[found].rx >= 240)) {
    resis[found].lpre = -4;
    resis[found].lx = (resis[found].lx + 5) / 10;
  } 

  ADC_DDR =  TXD_MSK;
  R_DDR = 0;

#endif
  return;
}  // end ReadInductance()

#if FLASHEND > 0x1fff
uint16_t get_log(uint16_t permil) {
  #define Log_Tab_Distance 20

  uint16_t y1, y2;
  uint16_t result;
  uint8_t tabind;
  uint8_t tabres;

  tabind = permil / Log_Tab_Distance;
  tabres = permil % Log_Tab_Distance;

  y1 = pgm_read_word(&LogTab[tabind]);
  y2 = pgm_read_word(&LogTab[tabind+1]);

  result = ((y2 - y1) * tabres ) / Log_Tab_Distance + y1;
  return(result);
}
#endif

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

#define MAX_CNT 255

#ifdef ADC_Sleep_Mode
  #define StartADCwait() sleep_cpu()
#else
  #define StartADCwait() ADCSRA = StartADCmsk; \
  while (ADCSRA & (1 << ADSC))
#endif

#if F_CPU == 16000000UL
  #define DelayBigCap() delayMicroseconds(20)
#else
  #define DelayBigCap() wait10us(); wait5us(); wait4us()
#endif

// FUNCTION: GetESR - Estimate equivalent series resistance for low-impedance components.
uint16_t GetESR(uint8_t hipin, uint8_t lopin) {
#if FLASHEND > 0x1fff
  unsigned int adcv[4];
  unsigned long sumvolt[4];
  unsigned long cap_val_nF;
  uint16_t esrvalue;
  uint8_t HiPinR_L;
  uint8_t HiADC;
  uint8_t LoPinR_L;
  uint8_t LoADC;
  uint8_t ii;
  uint8_t StartADCmsk;
  uint8_t SelectLowPin,SelectHighPin;
  int8_t esr0;

  StartADCmsk = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV;

  if (PartFound == PART_CAPACITOR) {
    ii = cap.cpre_max;
    cap_val_nF = cap.cval_max;

    while (ii < -9) {
      cap_val_nF /= 10;
      ii++;
    }

    if (cap_val_nF < (1800/18)) return(0xffff);
  }

  LoADC = pgm_read_byte(&PinADCtab[lopin]) | TXD_MSK;
  HiADC = pgm_read_byte(&PinADCtab[hipin]) | TXD_MSK;
  LoPinR_L = pgm_read_byte(&PinRLtab[lopin]);
  HiPinR_L = pgm_read_byte(&PinRLtab[hipin]);

  #if PROCESSOR_TYP == 1280
    SelectLowPin = (lopin | (1<<REFS1) | (0<<REFS0));
    SelectHighPin = (hipin | (1<<REFS1) | (0<<REFS0));
  #else
    SelectLowPin = (lopin | (1<<REFS1) | (1<<REFS0));
    SelectHighPin = (hipin | (1<<REFS1) | (1<<REFS0));
  #endif

  sumvolt[0] = 1;
  sumvolt[2] = 1;
  sumvolt[1] = 0;
  sumvolt[3] = 0;
  EntladePins();
  ADC_PORT = TXD_VAL;
  ADMUX = SelectLowPin;

  #ifdef NO_AREF_CAP
    wait250us();
  #else
    wait_about10ms();
  #endif

  ADC_DDR = HiADC;
  R_PORT = LoPinR_L;
  R_DDR = LoPinR_L;
  wait10us();
  wait2us();
  R_DDR = 0;
  R_PORT = 0;
  StartADCwait();

  for(ii=0;ii<MAX_CNT;ii++) {
    ADC_DDR = LoADC;
    R_PORT = LoPinR_L;
    R_DDR = LoPinR_L;
    ADMUX = SelectLowPin;
    StartADCwait();
    StartADCwait();
    adcv[0] = ADCW;
    ADMUX = SelectHighPin;

    StartADCwait();
    ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV;
    wait4us();
    R_PORT = HiPinR_L;
    R_DDR = HiPinR_L;
    DelayBigCap();

    R_DDR = 0;
    R_PORT = 0;
    while (ADCSRA&(1<<ADSC));
    adcv[1] = ADCW;

    wdt_reset();

    ADC_DDR = HiADC;
    R_PORT = HiPinR_L;
    R_DDR = HiPinR_L;
    wdt_reset();
    ADMUX = SelectHighPin;
    StartADCwait();
    StartADCwait();
    adcv[2] = ADCW;
    ADMUX = SelectLowPin;

    StartADCwait();
    ADCSRA = (1<<ADSC) | (1<<ADEN) | (1<<ADIF) | AUTO_CLOCK_DIV;
    wait4us();
    R_PORT = LoPinR_L;
    R_DDR = LoPinR_L;
    DelayBigCap();

    R_DDR = 0;
    R_PORT = 0;

    while (ADCSRA&(1<<ADSC));
    adcv[3] = ADCW;

    sumvolt[0] += adcv[0];
    sumvolt[1] += adcv[1];
    sumvolt[2] += adcv[2];
    sumvolt[3] += adcv[3];
  }

  sumvolt[0] += sumvolt[2];

  if ((sumvolt[1] + sumvolt[3]) > sumvolt[0]) {
    sumvolt[2] = (sumvolt[1] + sumvolt[3]) - sumvolt[0];
  } else {
    sumvolt[2] = 0;
  }

  if (PartFound == PART_CAPACITOR) {
    sumvolt[2] -= (1745098UL*MAX_CNT) / (cap_val_nF * (cap_val_nF + 19));
  }

  esrvalue = (sumvolt[2] * 10 * (unsigned long)RRpinMI) / (sumvolt[0]+sumvolt[2]);
  esrvalue += esrvalue / 14;
  esr0 = (int8_t)pgm_read_byte(&EE_ESR_ZEROtab[hipin+lopin]);

  if (esrvalue > esr0) {
    esrvalue -= esr0;
  } else {
    esrvalue = 0;
  }

  return (esrvalue);
#else
  return (0);
#endif
}

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// FUNCTION: GetVloss - Estimate capacitor self-discharge losses over a controlled delay.
void GetVloss() {
#if FLASHEND > 0x1fff
  unsigned int tmpint;
  unsigned int adcv[4];

  union t_combi{
  unsigned long dw;
  uint16_t w[2];
  } lval;

  uint8_t ii;
  uint8_t HiPinR_L;
  uint8_t LoADC;

  if (cap.v_loss > 0) return;

  LoADC = pgm_read_byte(&PinADCtab[cap.ca]) | TXD_MSK;
  HiPinR_L = pgm_read_byte(&PinRLtab[cap.cb]);

  EntladePins();
  ADC_PORT = TXD_VAL;
  R_PORT = 0;
  ADC_DDR = LoADC;
  R_DDR = HiPinR_L;
  adcv[0] = ReadADC(cap.cb);

  if (cap.cpre_max > -9) return;

  lval.dw = cap.cval_max;
  for (ii=cap.cpre_max+12;ii<4;ii++) {
    lval.dw = (lval.dw + 5) / 10;
  }

  if ((lval.dw == 0) || (lval.dw > 5000)) {
    return;
  }

  R_PORT = HiPinR_L;
  R_DDR = HiPinR_L;

  for (tmpint=0; tmpint<lval.w[0]; tmpint+=2) {
    wait5us();
  }

  R_DDR = 0;
  R_PORT = 0;
  wdt_reset();

  ADCconfig.Samples = 5;
  adcv[2] = ReadADC(cap.cb);
  if (adcv[2] > adcv[0]) {
    adcv[2] -= adcv[0];
  } else {
    adcv[2] = 0;
  }

  for (tmpint=0; tmpint<lval.w[0]; tmpint++) {
    wait5us();
  }

  adcv[3] = ReadADC(cap.cb);
  ADCconfig.Samples = ANZ_MESS;
  wdt_reset();

  if (adcv[3] > adcv[0]) {
    adcv[3] -= adcv[0];
  } else {
    adcv[3] = 0;
  }

  if (adcv[2] > adcv[3]) {
    adcv[1] = adcv[2] - adcv[3];
  } else {
    adcv[1] = 0;
  }

  if (adcv[1] > 0) {
    cap.v_loss = (unsigned long)(adcv[1] * 500UL) / adcv[2];
  }

  EntladePins();

  ADC_DDR =  TXD_MSK;
  ADC_PORT = TXD_VAL;
  R_DDR = 0;
  R_PORT = 0;

#endif
  return;
}  // end GetVloss()

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// FUNCTION: Calibrate_UR - Read the internal reference and apply the enabled ADC calibration.
void Calibrate_UR(void) {
  #ifdef AUTO_CAL
    uint16_t sum_rm;
    uint16_t sum_rp;
    uint16_t u680;
  #endif

  ADCconfig.U_AVCC = U_VCC;
  ADCconfig.Samples = 200;

  #if FLASHEND > 0x1fff
    ADC_PORT = TXD_VAL;
    ADC_DDR = (1<<TPREF) | TXD_MSK;
    wait1ms();
    ADC_DDR =  TXD_MSK;
    trans.uBE[1] = W5msReadADC(TPREF);

    if ((trans.uBE[1] > 2250) && (trans.uBE[1] < 2750)) {
      WithReference = 1;
      ADCconfig.U_AVCC = (unsigned long)((unsigned long)ADCconfig.U_AVCC * 2495) / trans.uBE[1];
    }
  #endif

  #ifdef WITH_AUTO_REF
    (void) ReadADC(MUX_INT_REF);
    ref_mv = W5msReadADC(MUX_INT_REF);
    RefVoltage();
  #else
    ref_mv = DEFAULT_BAND_GAP;
  #endif
 
  ADCconfig.U_Bandgap = ADC_internal_reference;

  #ifdef AUTO_CAL
    ADC_DDR = 1<<TP1 | TXD_MSK;
    R_PORT = 1<<(TP1*2);
    R_DDR = 1<<(TP1*2);
    sum_rm = W5msReadADC(TP1);

    ADC_DDR = 1<<TP2 | TXD_MSK;
    R_PORT =  1<<(TP2*2);
    R_DDR = 1<<(TP2*2);
    sum_rm += W5msReadADC(TP2);

    ADC_DDR = 1<<TP3 | TXD_MSK;
    R_PORT =  1<<(TP3*2);
    R_DDR = 1<<(TP3*2);
    sum_rm += W5msReadADC(TP3);

    R_PORT = 0;
    ADC_PORT = 1<<TP1 | TXD_VAL;
    ADC_DDR = 1<<TP1 | TXD_MSK;
    R_DDR = 1<<(TP1*2);
    sum_rp = ADCconfig.U_AVCC - W5msReadADC(TP1);
      
    ADC_PORT = 1<<TP2 | TXD_VAL;
    ADC_DDR = 1<<TP2 | TXD_MSK;
    R_DDR = 1<<(TP2*2);
    sum_rp += ADCconfig.U_AVCC - W5msReadADC(TP2);

    ADC_PORT = 1<<TP3 | TXD_VAL;
    ADC_DDR = 1<<TP3 | TXD_MSK;
    R_DDR = 1<<(TP3*2);
    sum_rp += ADCconfig.U_AVCC - W5msReadADC(TP3);

    u680 = ((ADCconfig.U_AVCC * 3) - sum_rm - sum_rp);
    pin_rmi = (unsigned long)((unsigned long)sum_rm * (unsigned long)R_L_VAL) / (unsigned long)u680;
    pin_rpl = (unsigned long)((unsigned long)sum_rp * (unsigned long)R_L_VAL) / (unsigned long)u680;
    resis680pl = pin_rpl + R_L_VAL;
    resis680mi = pin_rmi + R_L_VAL;
  #endif

  ADCconfig.Samples = ANZ_MESS;
} 

/* -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- -=- */

// SECTION: Display adapters for DisplayValue & legacy portability.
void lcd_set_cursor(uint8_t row, uint8_t col)
{
  #ifdef LCD1602
    int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if ( row >= 2 ) {
      row = 1;
    }
    lcd.command(CMD_SetDDRAMAddress | (col + row_offsets[row]));
  #endif

  #ifdef NOK5110
    lcd.setCursor(6*col, 10*row);
  #endif

  #ifdef OLED096
    display.setCursor(6*col, MG_OLED_TOP_OFFSET + 10*row);
  #endif

  uart_newline();
}

void lcd_string(char *data) {
  while(*data) {
    lcd_data(*data);
    data++;
  }
}

void lcd_pgm_string(const unsigned char *data) {
  unsigned char cc;
  while(1) {
    cc = pgm_read_byte(data);
    if((cc == 0) || (cc == 128)) return;
    lcd_data(cc);
    data++;
  }
}

#ifdef LCD1602
void lcd_pgm_custom_char(uint8_t location, const unsigned char *chardata) {	
  location &= 0x7;
  lcd.command(CMD_SetCGRAMAddress | (location << 3));
  for(uint8_t i=0;i<8;i++) {
    lcd.write(pgm_read_byte(chardata));
    chardata++;
  }
}
#endif

void lcd_testpin(unsigned char temp) {
  lcd_data(temp + '1');
}

void lcd_space(void) {
  lcd_data(' ');
}

void lcd_fix_string(const unsigned char *data) {
  unsigned char cc;
  while(1) {
    cc = MEM_read_byte(data);
    if((cc == 0) || (cc == 128)) return;
    lcd_data(cc);
    data++;
  }
}

void lcd_data(unsigned char temp1) {
  #ifdef LCD1602
    lcd.write(temp1);
  #endif

  #ifdef NOK5110
    lcd.write(temp1);
  #endif

  #ifdef OLED096
    display.write(temp1);
  #endif

  #ifdef WITH_UART
  switch(temp1) {
    case LCD_CHAR_DIODE1: uart_putc('>'); uart_putc('|'); break;
    case LCD_CHAR_DIODE2: uart_putc('|'); uart_putc('<'); break;
    case LCD_CHAR_CAP:    uart_putc('|'); uart_putc('|'); break;
    case LCD_CHAR_RESIS1: uart_putc('['); uart_putc('='); break;
    case LCD_CHAR_RESIS2: uart_putc(']'); break;
    case LCD_CHAR_U:      uart_putc('u'); break;
    case LCD_CHAR_OMEGA:  uart_putc('o'); uart_putc('h'); uart_putc('m'); break;
    default:              uart_putc(temp1);
  }
  #endif
}

void lcd_clear(void) {
  #ifdef LCD1602
    lcd.clear();
  #endif

  #ifdef NOK5110
    lcd.clearDisplay();
  #endif

  #ifdef OLED096
    display.clearDisplay();
  #endif

  uart_newline();
}

#ifdef WITH_UART
void uart_putc(uint8_t data) {
  Serial.write(data);
}
#endif
