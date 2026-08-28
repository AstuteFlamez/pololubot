# capstone/testdata — the telemetry-CSV contract fixture and its check.
#
# `make check` asks one question: can the telemetry pipeline decode
# smoke.csv end to end? The fixture is written the way the POST-T1
# firmware dumps a run (snapshot kinds J/A, EV_RESUME) — so this target is
# RED until F1b/CODE-7 land, and stays green forever after as the CSV
# contract's regression test (invariant I3: kinds are append-only).
#
# smoke.csv is SYNTHETIC — hand-written to that format, not captured from a
# robot, and its line 1 says so. It proves the decode chain, never a
# hardware outcome.
#
# Deliberately separate from capstone/host_tests/Makefile: that suite
# builds logic/ with cc and has its own red/green story; this check needs
# only python3 + stdlib (no matplotlib, no venv).

check:
	python3 check_decode.py

.PHONY: check
