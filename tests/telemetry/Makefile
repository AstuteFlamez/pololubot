# The telemetry-CSV fixture and its decode check.
#
# `make check` asks one question: does smoke.csv decode end to end through
# tools/plot_telemetry.py? The fixture is written the way the firmware
# dumps a run (snapshot tick kinds J/A, the EV_RESUME event), so this is
# the regression test for the CSV format — whose central rule is that tick
# kinds and event codes are append-only, because renumbering one silently
# re-labels every capture taken before the change.
#
# smoke.csv is SYNTHETIC — hand-written to that format, not captured from a
# robot, and its line 1 says so. It proves the decode chain, never a
# hardware outcome.
#
# Deliberately separate from tests/logic/Makefile: that suite needs a C
# compiler to build logic/; this check needs only python3 and the standard
# library (no matplotlib, no venv).

check:
	python3 check_decode.py

.PHONY: check
