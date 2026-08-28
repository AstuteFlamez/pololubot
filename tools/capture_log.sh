#!/usr/bin/env bash
# capture_log.sh — capture a LOG DUMP off the robot into logs/, hands-free.
#
# Why a script: the capture terminal has to be RUNNING before you trigger
# the dump (miss the first lines and the CSV is headerless), and a session
# has better things to hold in its head than stty incantations. This does
# the whole ritual: finds the robot's USB serial port, opens a timestamped
# file in logs/, tells you which buttons to press, and stops by itself when
# it sees the '# end' terminator.
#
# Usage:
#   capstone/capture_log.sh              -> logs/20260810_143200.csv
#   capstone/capture_log.sh V2_straight  -> logs/20260810_143200_V2_straight.csv
#
# logs/ is expendable scratch (gitignored). If the run turns out to be a
# ladder stage's exit PASS, COPY it into capstone/validation/ under the
# permanent naming scheme: NNN_<stage>_<what>.csv (see validation/README.md).
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
LOGS="$REPO/logs"

# ---- find the robot -------------------------------------------------------
# The 3pi+ 2040 enumerates as a USB CDC serial port: /dev/cu.usbmodem*.
# CAPTURE_PORT_GLOB exists so the no-port error path can be TESTED without
# unplugging anything (testability is a design requirement, not a luxury):
#   CAPTURE_PORT_GLOB='/dev/cu.nosuchport*' capstone/capture_log.sh
ports=(${CAPTURE_PORT_GLOB:-/dev/cu.usbmodem*})
if [ ! -e "${ports[0]}" ]; then
    echo "capture_log.sh: no /dev/cu.usbmodem* port found." >&2
    echo "  - Is the robot plugged in over USB-C?"            >&2
    echo "  - Is its power switch ON? (The ring lives in SRAM — if you" >&2
    echo "    powered off after the run, the log is already gone.)"     >&2
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
echo "next: .venv/bin/python3 capstone/plot_telemetry.py '$out' --summary"
echo "stage-exit PASS? copy it: cp '$out' capstone/validation/NNN_<stage>_<what>.csv"
