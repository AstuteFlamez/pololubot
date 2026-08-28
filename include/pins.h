// pins.h — 3pi+ 2040 pin cheat sheet.
// Every mapping verified against Pololu's own C library source
// (reference/pololu-3pi-2040-robot/c/) and the user's guide (0J86).
//
//  Function              GPIO      Peripheral / notes
//  -------------------   -------   -----------------------------------------
//  Yellow LED            GP25      ACTIVE LOW. Shared with button A!
//  Button A              GP25      read via output-enable-override trick
//  Button B              QSPI_SS   the BOOTSEL pin — not a normal GPIO
//  Button C              GP0       press = LOW (pull-up). Shared w/ OLED D/C!
//  OLED D/C              GP0       SH1106 display, bit-banged/SPI0
//  OLED RESET            GP1
//  OLED CLK              GP2       SPI0 SCK
//  OLED MOSI / RGB data  GP3       SPI0 TX — shared between display & LEDs
//  IMU SDA               GP4       i2c0 — LSM6DSO (accel+gyro) @ 0x6B,
//  IMU SCL               GP5       LIS3MDL (magnetometer) @ 0x1E
//  RGB LED clock         GP6       6× APA102-style addressable LEDs
//  Buzzer                GP7       PWM slice 3, channel B
//  Right encoder A/B     GP8/GP9   quadrature, 12 counts/motor-rev
//  Right motor DIR       GP10      HIGH = reverse
//  Left  motor DIR       GP11      HIGH = reverse
//  Left encoder A/B      GP12/GP13 quadrature (~358 counts/wheel-rev @30:1)
//  Right motor PWM       GP14      PWM slice 7 ch A, wrap 6000 → 20.8 kHz
//  Left  motor PWM       GP15      PWM slice 7 ch B
//  Bump right            GP16      IR RC-decay sensor (like line sensors)
//  Bump left             GP17
//  Line sensors 5..1     GP18–GP22 down-facing IR, RC-decay timing;
//                                  GP22 is leftmost (DN1) … GP18 rightmost (DN5)
//  Bump emitter ctl      GP23      HIGH = front IR emitters on
//  Line emitter ctl      GP26      HIGH = down IR emitters on … AND:
//  Battery monitor       GP26/ADC0 reads VBAT/11 through a divider (!)
//
// Yes, GP26 has two jobs, and three pins are double-booked (GP25, GP0, GP3).
// When a conflict finally bites you (~Day 9, display + button C), open the
// schematic and figure out WHY it works anyway. That's the lesson.
//
// Big picture memory map (RP2040 datasheet §2.2):
//   0x00000000  boot ROM
//   0x10000000  XIP flash (your code lives here, executed in place)
//   0x20000000  SRAM (264 KB)
//   0x40000000  APB peripherals (IO_BANK0, PADS, TIMER, PWM, ADC, I2C…)
//   0xD0000000  SIO — single-cycle GPIO in/out/set/clr/xor (Day 2 home)

#ifndef PINS_H
#define PINS_H

#define PIN_LED           25
#define PIN_BUTTON_C      0
#define PIN_OLED_DC       0
#define PIN_OLED_RESET    1
#define PIN_OLED_CLK      2
#define PIN_OLED_MOSI     3
#define PIN_RGB_DATA      3
#define PIN_IMU_SDA       4
#define PIN_IMU_SCL       5
#define PIN_RGB_CLK       6
#define PIN_BUZZER        7
#define PIN_ENC_RIGHT_A   8
#define PIN_ENC_RIGHT_B   9
#define PIN_MOTOR_RIGHT_DIR 10
#define PIN_MOTOR_LEFT_DIR  11
#define PIN_ENC_LEFT_A    12
#define PIN_ENC_LEFT_B    13
#define PIN_MOTOR_RIGHT_PWM 14
#define PIN_MOTOR_LEFT_PWM  15
#define PIN_BUMP_RIGHT    16
#define PIN_BUMP_LEFT     17
#define PIN_LINE_5        18
#define PIN_LINE_4        19
#define PIN_LINE_3        20
#define PIN_LINE_2        21
#define PIN_LINE_1        22
#define PIN_BUMP_EMITTER  23
#define PIN_LINE_EMITTER  26
#define ADC_BATTERY       0     // ADC input 0 = GP26, reads VBAT/11

#endif
