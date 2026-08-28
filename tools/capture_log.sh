#!/usr/bin/env bash
# capture_log.sh — capture a LOG DUMP off the robot into logs/, hands-free.
#
# Why a script: the capture has to be RUNNING before the dump is triggered
# (miss the first lines and the CSV is headerless), and the stty
# incantation is not worth memorizing. This does the whole procedure: finds
# the robot's USB serial port, opens a timestamped file in logs/, prints
# which buttons to press, and stops by itself at the '# end' terminator.
# The CSV format itself is specified in src/telemetry.h.
#
# Usage:
#   tools/capture_log.sh              -> logs/20260810_143200.csv
#   tools/capture_log.sh straight     -> logs/20260810_143200_straight.csv
#
# TWO WAYS TO LOSE A RUN, both of them permanent:
#
#   1. The ring lives in SRAM, which forgets everything the instant power
#      drops. Do not switch the robot off between finishing the run and
#      dumping the log — battery switch stays ON for the whole walk back to
#      the bench. Power off first and the last ~16 seconds of the run are
#      gone; not corrupted, never written anywhere but volatile memory.
#   2. Every GO wipes the ring. telemetry_reset() runs at the start of BOTH
#      runners (explore and replay), so an explore worth reading is erased
#      the moment GO is pressed on the replay that follows it. One run, one
#      dump, in that order — an explore-then-replay demo yields TWO CSVs,
#      never one.
#
# The dump only ever holds the most recent ~16 seconds. On a longer run the
# ring has already overwritten the early part, so plan the capture around
# the segment that matters (stop the robot with a button right after the
# moment of interest rather than letting it run on).
#
# The procedure this automates, by hand: find the port with
# `ls /dev/cu.usbmodem*`; start the capture FIRST with
# `(stty raw; cat) < /dev/cu.usbmodemXXXX > run1.csv` (or
# `screen -L -Logfile run1.csv /dev/cu.usbmodemXXXX`); then, on the robot's
# menu, C-tap to LOG DUMP and C-hold to GO; wait — ~8000 records over USB
# serial takes several seconds; stop the capture when the '# end' line
# appears.
#
# logs/ is expendable scratch (gitignored): overwrite it, delete it.
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
LOGS="$REPO/logs"

# ---- find the robot -------------------------------------------------------
# The 3pi+ 2040 enumerates as a USB CDC serial port: /dev/cu.usbmodem*.
# CAPTURE_PORT_GLOB exists so the no-port error path can be tested without
# unplugging anything:
#   CAPTURE_PORT_GLOB='/dev/cu.nosuchport*' tools/capture_log.sh
ports=(${CAPTURE_PORT_GLOB:-/dev/cu.usbmodem*})
if [ ! -e "${ports[0]}" ]; then
    echo "capture_log.sh: no /dev/cu.usbmodem* port found." >&2
    echo "  - Is the robot plugged in over USB-C?"            >&2
    echo "  - Is its power switch ON? (The ring lives in SRAM: if the"  >&2
    echo "    robot was powered off after the run, the log is gone.)"    >&2
    echo "  - Give macOS a second or two after plugging in, then retry." >&2
    exit 1
fi
port="${ports[0]}"
if [ "${#ports[@]}" -gt 1 ]; then
    echo "note: ${#ports[@]} usbmodem ports found; using $port" >&2
fi

# ---- open the output file -------------------------------------------------
mkdir -p "$LOGS"
stamp="$(date +%Y%m%d_%H%M%S)"
label="${1:+_$1}"
out="$LOGS/${stamp}${label}.csv"

# Raw mode: no echo, no line-ending rewrites — the CSV must arrive exactly
# as the firmware printed it. (Baud is nominal; USB CDC ignores it.)
stty -f "$port" raw 115200 2>/dev/null || true

echo "capturing from $port"
echo "  -> $out"
echo ""
echo "On the robot: C-tap to LOG DUMP, then C-hold to GO."
echo "The dump takes several seconds. Capture stops by itself at the"
echo "'# end' line. Ctrl-C abandons (partial file stays in logs/)."
echo ""

# ---- capture until the terminator -----------------------------------------
lines=0
while IFS= read -r line; do
    line="${line%$'\r'}"                # CDC sends CRLF; the CSV wants LF
    printf '%s\n' "$line" >> "$out"
    lines=$((lines + 1))
    if [ $((lines % 1000)) -eq 0 ]; then
        echo "  ... $lines lines"
    fi
    case "$line" in
        "# end"*) break ;;
    esac
done < "$port"

echo ""
echo "done: $lines lines -> $out"
echo "next: .venv/bin/python3 tools/plot_telemetry.py '$out' --summary"
