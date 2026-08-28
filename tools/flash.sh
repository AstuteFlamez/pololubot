#!/bin/bash
# flash.sh <file.uf2> — upload a program to the 3pi+ 2040 over USB.
#
# How this works: the RP2040's boot ROM contains a USB bootloader (BOOTSEL
# mode) that shows up as a fake flash drive named RPI-RP2. Copy a .uf2 file
# onto it and the ROM writes your program into flash and reboots. Two ways in:
#   1. Our programs all enable USB stdio; opening their serial port at exactly
#      1200 baud is a magic knock that reboots into BOOTSEL. (This script.)
#   2. Hardware fallback that ALWAYS works, even with broken firmware:
#      hold button B (the BOOTSEL pin!) while pressing RESET.
set -e
UF2="$1"
[ -f "$UF2" ] || { echo "usage: flash.sh <file.uf2>"; exit 1; }

VOL=/Volumes/RPI-RP2

if [ ! -d "$VOL" ]; then
  P=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
  if [ -n "$P" ]; then
    echo "1200-baud knock on $P ..."
    python3 - "$P" <<'EOF'
import termios, time, os, sys
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
a[4] = a[5] = termios.B1200
termios.tcsetattr(fd, termios.TCSANOW, a)
time.sleep(0.2)
os.close(fd)
EOF
  else
    echo "No serial port found. Hold button B, press RESET, then rerun."
  fi
  for _ in $(seq 1 30); do
    [ -d "$VOL" ] && break
    sleep 0.5
  done
fi

[ -d "$VOL" ] || { echo "RPI-RP2 drive never appeared. Hold button B + press RESET, rerun."; exit 1; }

cp "$UF2" "$VOL/"
echo "flashed $(basename "$UF2") — robot is rebooting into your code"
