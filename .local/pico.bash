set -e

cd ..
make sine_pico_default

picotool load -x build/sine_pico_default/sine_pico_default.elf
