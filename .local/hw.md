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

## Serial port -> ttySx mapping

NuttX assigns /dev/ttySx by walking a fixed-position array (USART1, USART2,
USART3, UART4, UART5, USART6, ...) and numbering only the *enabled* ones in
order, skipping disabled slots (gaps aren't reserved). So ttySx is NOT the
same as the peripheral number - it depends on which ports are enabled in
nuttx-config/nsh/defconfig.

Source: arm_serialinit() in
platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32/stm32_serial.c

Current board config (USART1/2/3/6 enabled, UART4/5 disabled):

| Peripheral | ttySx  |
|------------|--------|
| USART1     | ttyS0  |
| USART2     | ttyS1  |
| USART3     | ttyS2  |
| USART6     | ttyS3  |

CONFIG_BOARD_SERIAL_* role assignments in default.px4board must match this
table (re-check whenever defconfig's enabled UART/USART set changes).


# Upload

PX4 uploader needs valid PX4 bootloader to accept FW.

Upload bootloader from similar board in DFU mode.

Prepare:
```bash
sudo apt install dfu-util
cd boards/speedybee/f405v3/bootloader/
arm-none-eabi-objcopy -I ihex -O binary flywoo_gnf405_bl_15d91db.hex speedybee_bootloader.bin
```

Hold BOOT0 button and reset power. Then upload:
```bash
dfu-util -a 0 --dfuse-address 0x08000000 -D speedybee_bootloader.bin
```

After reboot you can upload FW normally:
```bash
make speedybee_f405v3_default upload
```
