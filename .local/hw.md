# Target

SpeedyBee F405 V3

MCU: STM32F405
Baro: SPL06
GYRO: BMI270
MAG: -/- (SDA/SCL)

## Pinout (derived from Betaflight unified-target + ArduPilot hwdef, cross-checked)

Not yet verified against schematic/hardware. Treat as a starting point.

- Gyro/Accel (BMI270): SPI1, CS PA4, EXTI PC4
- Baro (SPL06): I2C2, addr 0x76, SCL PB10, SDA PB11
- OSD (MAX7456, unused for PX4): SPI2, CS PB12
- SD card: SPI2, CS PA15
- UART1: TX PA9, RX PA10
- UART2: TX PA2, RX PA3
- UART3: TX PC10, RX PC11
- UART4: TX PA0, RX PA1
- UART5: TX PC12, RX PD2
- UART6: TX PC6, RX PC7
- Motor/PWM: PB6, PB7, PB8, PB9 (TIM4), PB0, PB1, PB5, PB4 (TIM3), PC9 (TIM8, LED strip pin also usable as PWM)
- ADC: battery voltage PC0, battery current PC1, RSSI PC2
- LED (status): PC8
- Beeper: PC5 (inverted)

Sources:
- https://github.com/betaflight/unified-targets/blob/master/configs/default/SPBE-SPEEDYBEEF405V3.config
- https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/speedybeef4v3/hwdef.dat

## Plan

1. Scaffold `boards/speedybee/f405-v3/` using `boards/omnibus/f4sd` as the closest
   existing PX4 template (same STM32F405, similar AIO FC layout).
2. Adapt `nuttx-config/` (defconfig, board.h, script.ld) for this MCU/flash layout.
3. Write `src/board_config.h`, `spi.cpp`, `i2c.cpp`, `init.c`, `led.c`, `timer_config.cpp`
   with the pinout above.
4. Set `default.px4board` sensor drivers: bmi270, spl06, no mag.
5. Build `make speedybee_f405-v3_default`, fix errors iteratively.
6. Flash and verify boot over USB/serial before wiring up sensors for real.

---

# Guide

https://docs.px4.io/main/en/hardware/porting_guide_config

## Board definition

boards/speedybee/f405v3/default.px4board

What features/modules to include.

# Vendor/Product
1209:5741
