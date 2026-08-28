#!/usr/bin/env python3
"""check_decode.py — does the telemetry CSV fixture decode end to end?

smoke.csv is the fixture that pins the telemetry CSV format, and this
script checks both ends of that format:

  * the host decoder (tools/plot_telemetry.py) must parse the fixture
    without errors and must treat the snapshot rows J and A as TICK rows
    with their own phase names — and must NOT let them open phase-shading
    spans, because a snapshot is an instant, not a phase, and shading a
    one-row "span" would paint time on the plot that the robot never
    spent in any phase;
  * the firmware encoder (src/telemetry.h and src/telemetry.c, read as
    text rather than executed) must be able to PRODUCE those rows at all.
    The plotter passes any event name through verbatim, so "UNK33" can
    only be born in the firmware's range checks and name tables.

The format is APPEND-ONLY: tick kinds and event codes may be added at the
end, never renumbered or reordered, because a renumbered code silently
re-labels every capture taken before the change. Several checks below
exist only to hold that line.

smoke.csv is a synthetic log dump of one junction pass — hand-written,
never captured. No robot produced those numbers, and none of them is
evidence of anything the hardware does. Its FORMAT is the firmware's: the
two one-shot snapshot tick kinds ('J' = kind 6, the edge-latched `before`
reading at JCT_DETECT; 'A' = kind 7, the at_center read at CREEP_END), the
EV_RESUME=33 event that marks the end of a backtrack recovery, the
self-identifying meta (the ops knob set on line 2; batt_mv= and the
calibration window on line 3) and a RESUME.b carrying the suppressed-resume
span in mm.

ONE DELIBERATE DEVIATION from the firmware's bytes, and it must survive
every future edit: line 1 carries a SYNTHETIC FIXTURE banner after the
version token. A shared CSV travels without its context, so a reader who
meets this file must not be able to mistake it for a real run. The banner
sits inside line 1's existing text on purpose — the format fixes the frame
at three '#' header lines, and the parser drops line 1's body wholesale
once it starts with "3pi2040" — so this is the one spot that carries prose
without breaking the frame. Do NOT promote it to a fourth header line.

Stdlib only, no matplotlib: plot_telemetry imports it lazily inside
plot_run(), so importing the module for its parser is safe on any machine.

Run:  make -C tests/telemetry check   (or: python3 check_decode.py)
"""
import io
import os
import re
import sys
import tempfile
from contextlib import redirect_stdout
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
SMOKE = HERE / "smoke.csv"

# Don't drop a __pycache__/ into tools/ as a side effect of importing the
# plotter — a checker that dirties the tree it checks is lying about scope.
sys.dont_write_bytecode = True
sys.path.insert(0, str(REPO / "tools"))
import plot_telemetry as pt  # noqa: E402  (module top is stdlib-only)

# The 17 event names the firmware dumped before RESUME was appended, in
# ring order. APPEND-ONLY: a renumbered or reordered name silently
# re-labels every CSV captured before the change, and history has to stay
# decodable.
ORIGINAL_EV_NAMES = [
    "RUN_START", "FOLLOW_START", "BLIND_END", "JCT_DETECT", "DEADEND",
    "LOSS", "BT_START", "BT_FOUND", "BT_FAIL", "CREEP_START", "CREEP_END",
    "TURN_START", "TURN_END", "CLASSIFY", "ABORT", "TIMEOUT", "FAULT",
]

_failures = 0


def check(ok, label, detail=""):
    global _failures
    if ok:
        print(f"  PASS  {label}")
    else:
        _failures += 1
        print(f"  FAIL  {label}" + (f"\n        -> {detail}" if detail else ""))


def main():
    # ---------------- host decoder: parse the fixture -----------------------
    print("== host decoder (plot_telemetry.py) vs smoke.csv ==")
    meta, ticks, events, raw_meta, errors, complete = pt.parse_csv(SMOKE)

    # The fixture is format-exact (3 '#' lines, 11 columns, '# end'); any
    # skipped line means the fixture or the parser broke the CSV contract.
    check(errors == 0, "fixture parses with zero errors",
          f"parse errors = {errors}")
    check(complete is True,
          "parser certifies the fixture's '# end' terminator (complete=True)",
          f"complete = {complete!r}")

    tick_letters_seen = {t["rec"] for t in ticks}
    event_names_seen = {e["name"] for e in events}

    # J/A are TICK rows: they carry real sensor values (the classifier's
    # actual evidence). Filed under events they lose their sensor columns
    # in every downstream tool.
    check({"J", "A"} <= tick_letters_seen,
          "J and A rows are classified as ticks",
          f"tick letters seen = {sorted(tick_letters_seen)}")
    check(not ({"J", "A"} & event_names_seen),
          "J and A rows are NOT mis-filed as events",
          f"event names seen include {sorted({'J','A'} & event_names_seen)}")
    check("J" in pt.TICK_LETTERS and "A" in pt.TICK_LETTERS,
          "TICK_LETTERS includes the snapshot kinds J and A",
          f"TICK_LETTERS = {sorted(pt.TICK_LETTERS)}")
    check(bool(pt.PHASE_NAME.get("J")) and bool(pt.PHASE_NAME.get("A")),
          "PHASE_NAME names both snapshot kinds",
          f"PHASE_NAME keys = {sorted(pt.PHASE_NAME)}")

    # Phase shading: spans mean "the robot SPENT TIME in this phase".
    # A snapshot row is one instant inside some other phase — letting it
    # open a span would shade time the robot never spent anywhere.
    # (Passes vacuously while J/A aren't ticks at all; it exists to bite in
    # the halfway state where TICK_LETTERS grew but the span filter did
    # not.)
    t0 = ticks[0]["t"] if ticks else 0
    spans = pt._phase_spans(ticks, lambda ms: (ms - t0) / 1000.0)
    span_letters = {s[2] for s in spans}
    check(not ({"J", "A"} & span_letters),
          "snapshot ticks are EXCLUDED from phase shading (_phase_spans)",
          f"span letters = {sorted(span_letters)}")
    check({"B", "C", "T", "K"} <= span_letters,
          "real phases still shade (fixture has B/C/T/K spans)",
          f"span letters = {sorted(span_letters)}")
    check(span_letters <= set(pt.PHASE_COLOR),
          "every span letter has a PHASE_COLOR (else plot_run crashes)",
          f"uncolored = {sorted(span_letters - set(pt.PHASE_COLOR))}")

    # The --summary ticks-by-phase table must count J/A like any tick kind.
    try:
        buf = io.StringIO()
        with redirect_stdout(buf):
            pt.print_summary(meta, ticks, events, raw_meta, errors)
        out = buf.getvalue()
        check(re.search(r"^\s*J\s*\(", out, re.M) is not None,
              "--summary ticks-by-phase lists J")
        check(re.search(r"^\s*A\s*\(", out, re.M) is not None,
              "--summary ticks-by-phase lists A")
        # Line 3's tokens hide behind print_summary's records= filter —
        # they must be surfaced explicitly or the reader never sees them.
        check("battery at dump" in out,
              "--summary surfaces battery from the dump-facts line")
        check("cal spans at dump" in out,
              "--summary surfaces cal spans from the dump-facts line")
    except Exception as e:  # a crash on the fixture is itself a finding
        check(False, "--summary runs on the fixture", f"raised {e!r}")

    check("RESUME" in event_names_seen,
          "fixture carries the RESUME event (fixture self-check)",
          "smoke.csv must model the post-fix dump")

    # ---- the dump must self-identify --------------------------------------
    # A CSV gets shared without its robot. If the file cannot name its own
    # knob values (line 2) and its battery and calibration window (line 3),
    # every triage question becomes "what was it built with?" —
    # unanswerable three flashes later. The fixture carries those tokens,
    # so they must parse back out of it.
    ops = ("replay", "arrival", "brake_mm", "brake_ms", "turn_kp",
           "dark_thresh", "goal_min_dark", "creep_mm", "think_ms",
           "cal_min_span")
    missing_ops = [t for t in ops if not isinstance(meta.get(t), int)]
    check(not missing_ops,
          "knob line names the ops knob set",
          f"missing/non-int tokens: {missing_ops}")
    check(isinstance(meta.get("batt_mv"), int),
          "dump-facts line carries batt_mv=",
          f"meta['batt_mv'] = {meta.get('batt_mv')!r}")
    spans = pt._cal_spans(meta)
    check(spans is not None and len(spans) == 5,
          "cal_min=/cal_max= parse to five per-sensor spans",
          f"_cal_spans -> {spans!r}")

    # ---------------- negative cases: the parser must OBJECT ----------------
    # A decode check that only ever eats a perfect fixture proves nothing
    # about the parser's defenses, so each variant below feeds it a
    # deliberate defect and asserts it FLAGS that defect. The variants are
    # built in memory from the REAL fixture and written to throwaway temp
    # files, because the parser's interface is a path; a permanent tracked
    # fixture per defect would be more than they are worth.
    print("== negative cases: parser vs deliberate defects ==")
    smoke_lines = SMOKE.read_text().splitlines(keepends=True)
    donor = "2388,B,48,300,812,300,56,0,0,1800,1800\n"
    check(donor in smoke_lines, "negative-case donor row present in fixture",
          "smoke.csv changed shape — update the donor row here")
    n_ticks_good = len(ticks)

    def parse_variant(lines):
        with tempfile.NamedTemporaryFile("w", suffix=".csv",
                                         delete=False) as f:
            f.writelines(lines)
            p = f.name
        try:
            return pt.parse_csv(p)
        finally:
            os.unlink(p)

    # (1) Malformed field: the ring is binary and the dump is printf, so a
    # non-numeric field means line noise hit the serial capture. The parser
    # must COUNT the row as an error and drop it — silently absorbing noise
    # would let a corrupted capture masquerade as a clean one.
    bad = [donor.replace("48,300", "4x,300") if l == donor else l
           for l in smoke_lines]
    _, tk, _, _, er, comp = parse_variant(bad)
    check(er == 1 and len(tk) == n_ticks_good - 1,
          "malformed field (non-int) is FLAGGED and only that row dropped",
          f"errors={er} (want 1), ticks={len(tk)} (want {n_ticks_good - 1})")
    check(comp is True,
          "malformed row does not cost the completeness verdict",
          f"complete = {comp!r}")

    # (2) Wrong column count (10 of 11 — the != 11 guard also catches 12).
    bad = ["2388,B,48,300,812,300,56,0,0,1800\n" if l == donor else l
           for l in smoke_lines]
    _, tk, _, _, er, _ = parse_variant(bad)
    check(er == 1 and len(tk) == n_ticks_good - 1,
          "wrong column count is FLAGGED and only that row dropped",
          f"errors={er} (want 1), ticks={len(tk)} (want {n_ticks_good - 1})")

    # (3) Clean truncation: capture stopped between rows, '# end' never
    # arrived. Every surviving row parses — so before the complete flag
    # existed, this file was INDISTINGUISHABLE from a whole dump (errors=0
    # for both). '# end' is the completeness receipt: its absence must be
    # detectable, and the rows must still be returned.
    _, tk, _, _, er, comp = parse_variant(smoke_lines[:-1])
    check(comp is False and er == 0 and len(tk) == n_ticks_good,
          "clean truncation (no '# end') is DETECTABLE: complete=False, "
          "surviving rows intact",
          f"complete={comp!r}, errors={er}, ticks={len(tk)}")

    # (4) Mid-row truncation: the capture died while a row was printing.
    # Both symptoms must show: the partial row flagged, completeness gone.
    cut = smoke_lines[:-2] + [smoke_lines[-2][:12]]
    _, _, _, _, er, comp = parse_variant(cut)
    check(comp is False and er == 1,
          "mid-row truncation: partial row FLAGGED and complete=False",
          f"complete={comp!r}, errors={er} (want 1)")

    # ---- saturation is judged PER RUN, from RUN_START.b --------------------
    # The one summary statistic here that a future edit could quietly undo.
    # "Saturated" means the PID asked for more steer than the base speed
    # can absorb, so the threshold is THIS run's base — which each run
    # announces in RUN_START.b. A replay lap steps that base above the meta
    # line's explore= speed, so judging it against the meta line
    # manufactures saturation out of perfectly healthy steering. The
    # regression is silent: every other check stays green while the number
    # lies, which is why it is asserted on every `make check`.
    print("== stepped-base guard: run judged against RUN_START.b ==")
    # Real header (so explore=1800 is the fixture's own knob line), then a
    # REPLAY run started at base 3500 whose five F ticks straddle BOTH
    # thresholds: all five clear 1800, only two clear 3500. The two
    # readings are 40.0% (honest) and 100.0% (the bug) — far enough apart
    # that no rounding can confuse them.
    stepped = smoke_lines[:4] + [
        "1000,RUN_START,0,0,0,0,0,4,3500,0,0\n"
    ] + [
        f"{1002 + 2 * i},F,48,300,812,300,56,{20 * i},{pid},3500,3500\n"
        for i, pid in enumerate((2000, 2400, 2900, 3600, 3700))
    ] + ["# end\n"]
    s_meta, s_ticks, s_events, s_raw, s_err, _ = parse_variant(stepped)
    buf = io.StringIO()
    with redirect_stdout(buf):
        pt.print_summary(s_meta, s_ticks, s_events, s_raw, s_err)
    sat_lines = [ln for ln in buf.getvalue().splitlines() if "saturated" in ln]
    check(re.search(r"^\s*run 1 \(REPLAY\): 5 ticks", buf.getvalue(), re.M)
          is not None,
          "F ticks are grouped under their own run (labelled by RUN_START)",
          "no per-run segment line — grouping regressed to one flat bucket")
    check(len(sat_lines) == 1 and "base=3500, RUN_START.b" in sat_lines[0]
          and "40.0% of ticks" in sat_lines[0],
          "stepped-base run: threshold is its own RUN_START.b",
          f"saturation line(s) = {sat_lines!r}")
    check(not any("explore" in ln or "100.0%" in ln for ln in sat_lines),
          "stepped-base run: the meta explore= reading is NOT what it prints",
          f"saturation line(s) = {sat_lines!r}")

    # ---------------- firmware encoder: static dump-layer check -------------
    # The plotter passes any event NAME through verbatim — "UNK33" can only
    # come from the firmware's dump tables (telemetry.c). So the honest test
    # of "RESUME decodes by name" is against those tables.
    print("== firmware encoder (src/telemetry.h/.c) vs smoke.csv ==")
    ht = (REPO / "src" / "telemetry.h").read_text()
    ct = (REPO / "src" / "telemetry.c").read_text()
    defines = {m.group(1): int(m.group(2)) for m in
               re.finditer(r"#define\s+((?:TEL_TICK|EV)_\w+)\s+(\d+)", ht)}

    check(defines.get("TEL_TICK_JCT_SNAP") == 6,
          "telemetry.h defines TEL_TICK_JCT_SNAP as kind 6",
          f"got {defines.get('TEL_TICK_JCT_SNAP')}")
    check(defines.get("TEL_TICK_CENTER_SNAP") == 7,
          "telemetry.h defines TEL_TICK_CENTER_SNAP as kind 7",
          f"got {defines.get('TEL_TICK_CENTER_SNAP')}")
    check(defines.get("EV_RESUME") == 33,
          "telemetry.h defines EV_RESUME as 33 (APPENDED after EV_FAULT)",
          f"got {defines.get('EV_RESUME')}")
    check(defines.get("TEL_TICK_FOLLOW") == 1 and defines.get("TEL_TICK_BT") == 5
          and defines.get("EV_RUN_START") == 16 and defines.get("EV_FAULT") == 32,
          "existing kind/event numbers unchanged (append-only)")

    # Letter map: dump prints "?FBCTK..."[kind] for tick rows.
    m = re.search(r'"\?FBCTK[A-Z]*"', ct)
    letter_map = m.group(0).strip('"') if m else ""
    check(len(letter_map) >= 8 and letter_map[6:8] == "JA",
          "dump letter map covers kinds 6/7 as J/A",
          f"letter map = {letter_map!r}")

    # Range checks: a kind outside them dumps as UNKn no matter what the
    # tables say. The EVENT bound is the easy one to forget.
    m = re.search(r"kind\s*<=\s*(TEL_TICK_\w+)", ct)
    tick_bound = defines.get(m.group(1)) if m else None
    check(tick_bound is not None and tick_bound >= 7,
          "tick range check admits kinds 6/7",
          f"upper bound resolves to {tick_bound} — kinds 6/7 dump as UNK6/UNK7")
    m = re.search(r"kind\s*<=\s*(EV_\w+)", ct)
    ev_bound = defines.get(m.group(1)) if m else None
    check(ev_bound is not None and ev_bound >= 33,
          "event range check admits EV_RESUME",
          f"upper bound resolves to {ev_bound} — kind 33 dumps as UNK33")

    # The dump-facts tokens the fixture models must be ones the firmware
    # actually prints — same pipeline honesty as the event-name check below.
    check("batt_mv=" in ct and "cal_min=" in ct and "cal_max=" in ct,
          "firmware dump prints the dump-facts tokens (batt_mv, cal window)")

    m = re.search(r"ev_names\[\]\s*=\s*\{(.*?)\};", ct, re.S)
    fw_names = re.findall(r'"(\w+)"', m.group(1)) if m else []
    check(fw_names[:17] == ORIGINAL_EV_NAMES,
          "existing ev_names unchanged, in order (old CSVs must decode)")
    check(len(fw_names) >= 18 and fw_names[17] == "RESUME",
          'ev_names decodes 33 by name: "RESUME" at index EV_RESUME-EV_RUN_START',
          f"ev_names has {len(fw_names)} entries: "
          f"{fw_names[17] if len(fw_names) > 17 else 'nothing'} at index 17")

    # Every rec name the fixture uses must be one the firmware can
    # actually emit: the fixture claims to be a dump, so it has to be a
    # dump the firmware could have produced.
    if ev_bound is not None and fw_names:
        emittable = {fw_names[k - 16] for k in
                     range(16, min(ev_bound, 15 + len(fw_names)) + 1)}
    else:
        emittable = set(fw_names)
    missing = sorted(event_names_seen - emittable)
    check(not missing,
          "every fixture event name is dumpable by the firmware",
          f"firmware cannot emit {missing} (would appear as UNKn)")
    if tick_bound is not None and letter_map:
        emittable_letters = set(letter_map[1:min(tick_bound, len(letter_map) - 1) + 1])
    else:
        emittable_letters = set(letter_map[1:])
    missing_l = sorted(tick_letters_seen - emittable_letters)
    check(not missing_l,
          "every fixture tick letter is dumpable by the firmware",
          f"firmware cannot emit tick letters {missing_l}")

    print()
    if _failures:
        print(f"DECODE CHECK: RED — {_failures} assertion(s) failing")
        return 1
    print("DECODE CHECK: GREEN — fixture decodes end to end")
    return 0


if __name__ == "__main__":
    sys.exit(main())
