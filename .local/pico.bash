set -e

cd ..
make raspberrypi_pico_default

picotool load -x build/raspberrypi_pico_default/raspberrypi_pico_default.elf
