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

# UARTS (SpeedyBee F405 V3)

# USART 1
TX    A09   GPIO_USART1_TX_1
RX    A10   GPIO_USART1_RX_1

Serial: /dev/ttyS0
Function: TEL1
Peripheral: GCS/WiFI

# USART 2
TX    A02   GPIO_USART2_TX_1
RX    A03   GPIO_USART2_RX_1

Serial: /dev/ttyS1
Function: TEL2
Perpheral: sine.link/beacon_ranges

# USART 3
TX    C10   GPIO_USART3_TX_2
RX    C11   GPIO_USART3_RX_2

Serial: /dev/ttyS2
Function: TEL3/Console
Perpheral: Optical Flow

# USART 6
TX    C06   GPIO_USART6_TX_1
RX    C07   GPIO_USART6_RX_1

Serial: /dev/ttyS3
Function: GPS
Perpheral: GPS


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


---

# Pico

## Picotool

Clone repos:
```bash
git clone https://github.com/raspberrypi/picotool.git

cd picotool

git clone https://github.com/raspberrypi/pico-sdk.git sdk
```

Build:
```bash
mkdir build
cd build

export PICO_SDK_PATH=../sdk

make raspberrypi_pico_default
cmake ..
make -j
```

Install:
```bash
sudo make install
```

## Build & upload

make raspberrypi_pico_default
picotool load -x -f build/raspberrypi_pico_default/raspberrypi_pico_default.elf

## Console

Only one USB port (CDC-ACM, `/dev/ttyACM0`), no dedicated UART console.
Connect with `screen /dev/ttyACM0` (baud is meaningless for USB CDC-ACM,
NuttX ignores it). No `make ... upload` target exists for this board (no
`boards/raspberrypi/pico/cmake/upload.cmake`) - flash with picotool as above.

Two bugs hit in a row getting a usable shell:

1. **Binary garbage interleaved with the NSH prompt.** Root cause:
   `boards/raspberrypi/pico/init/rc.board_mavlink` autostarted
   `mavlink start -d /dev/ttyACM0` - the exact same device as the NSH USB
   console. MAVLink's binary frames and NSH's text were multiplexed onto
   the same serial stream. Fixed by commenting out the autostart; run
   `mavlink start -d /dev/ttyACM0` by hand when actually needed.

2. **Missing CR before LF ("staircase" output, cursor drifts right every
   line).** Not a baud/terminal-emulator issue - confirmed via clean
   `journalctl -k` USB logs and identical corruption in both minicom and
   screen. Root cause: `CONFIG_INIT_ENTRYPOINT="nsh_main"` on a USB-console
   board resolves to `nsh_consolemain()` -> `nsh_waitusbready()`
   (`apps/nshlib/nsh_usbconsole.c`), which opens `/dev/ttyACM0` and `dup2`s
   it to stdio *without ever touching termios*. `\n` -> `\r\n` translation
   (`OPOST|ONLCR`) is normally set by `uart_register()`'s `isconsole`
   default, but the CDC-ACM device here isn't flagged as the system console
   (`CONFIG_CDCACM_CONSOLE` was unset, matching upstream's own
   `raspberrypi-pico/configs/usbnsh/defconfig`) - so nothing ever turns
   OPOST on for this path. `src/systemcmds/nshterm/nshterm.cpp` already
   works around this exact gap for its own code path (explicit
   `tcgetattr`/`tcsetattr` before starting a session), but
   `nsh_waitusbready()` doesn't. Patched `nsh_waitusbready()` to do the same
   fix - see `.local/nuttx-apps-nsh_usbconsole-crlf-fix.patch`.

   This lives in the `platforms/nuttx/NuttX/apps` submodule, so it's
   invisible to `git status`/commits in the main repo and will be wiped by
   a `git submodule update`. Re-apply from the patch file if that happens,
   or commit it inside the submodule properly once confirmed working.

   Likely affects any NuttX board using `CONFIG_NSH_USBCONSOLE` without
   `CONFIG_CDCACM_CONSOLE`, not just this one - worth a look upstream
   (apache/nuttx) if this comes up again.
