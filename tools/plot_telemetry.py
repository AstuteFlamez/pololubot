#!/usr/bin/env python3
"""plot_telemetry.py — turn a flight-recorder CSV dump into pictures.

The CSV format this reads is specified in src/telemetry.h; tools/
capture_log.sh captures a dump off the robot. Parsing and summarizing
need stdlib only — matplotlib is imported lazily, so `--summary` still
works on a machine that does not have it installed.

Usage:
    python3 plot_telemetry.py run1.csv                # writes run1.png
    python3 plot_telemetry.py run1.csv --out foo.png
    python3 plot_telemetry.py run1.csv --summary       # also prints stats
"""
import argparse
import sys
from collections import Counter
from pathlib import Path

TICK_LETTERS = set("FBCTKJA")
# J and A are ONE-SHOT snapshot ticks — the classifier's two actual
# inputs ('J' = the edge-latched `before` at JCT_DETECT, 'A' = the
# at_center money read at CREEP_END). They are instants, not phases:
# they carry sensor columns like any tick, but must never open a
# shading span (see _phase_spans).
SNAPSHOT_LETTERS = set("JA")
PHASE_NAME = {"F": "line-follow", "B": "blind", "C": "creep",
              "T": "turn", "K": "backtrack",
              "J": "jct snap", "A": "center snap"}
# TEL_TICK_* codes from src/telemetry.h — used to decode ABORT/TIMEOUT's
# "a" field (which tick phase it happened in). Snapshot kinds 6/7 can't
# appear there: an abort happens IN a phase, never in an instant.
TICK_CODE_LETTER = {1: "F", 2: "B", 3: "C", 4: "T", 5: "K"}
# junction_t from logic/maze_logic.h, in enum order — decodes CLASSIFY's a.
JCT_NAMES = ["LEFT_ONLY", "RIGHT_ONLY", "STRAIGHT_LEFT", "STRAIGHT_RIGHT",
             "T", "CROSS", "DEAD_END", "GOAL", "NONE"]
# run_mode_t from src/feedback.h, in enum order — decodes RUN_START's a.
# A maze run is always EXPLORE (2) or REPLAY (4); the others exist because
# the firmware logs whatever mode started the run, not the expected one.
MODE_NAMES = ["DIAG", "CALIBRATE", "EXPLORE", "SOLVED", "REPLAY", "DONE",
              "SPARE", "LOGDUMP"]

# A validated categorical order (see dataviz palette) — fixed slot order,
# never cycled or reassigned per-run.
SENSOR_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
DUTY_COLORS = ["#2a78d6", "#eb6834"]
# Only PHASES get shading colors. J/A are deliberately absent: a snapshot
# never opens a span, so it never needs one — adding a color here would
# invite someone to shade it.
PHASE_COLOR = {"B": "#4a3aa7", "C": "#008300", "T": "#eda100", "K": "#e34948"}
REF_COLOR = "#898781"      # muted ink — reference lines are annotation, not data
EVENT_COLOR = "#52514e"    # secondary ink — event markers are annotation


def _try_int(v):
    try:
        return int(v)
    except ValueError:
        return v


def parse_csv(path):
    """Parse the telemetry CSV. Returns (meta, ticks, events, raw_meta_line,
    n_errors, complete). Malformed lines are skipped and counted, never
    raise — the ring can be dumped mid-write and the file can be cut off
    anywhere. `complete` reports whether the `# end` terminator was seen:
    the dump prints it last, so its absence means the capture was cut off
    (Ctrl-C too early, cable yanked) and the tail of the run may be
    missing. Rows are still returned — a truncated dump is evidence, just
    evidence that must say so."""
    meta = {}
    raw_meta_line = None
    ticks, events = [], []
    errors = 0
    complete = False
    last_raw, offset = None, 0

    with open(path, "r", errors="replace") as f:
        for line in f:
            line = line.rstrip("\r\n")
            if not line:
                continue

            if line.startswith("#"):
                body = line[1:].strip()
                if not body or body.startswith("3pi2040"):
                    continue
                if body == "end":
                    complete = True
                    break
                if "=" in body:
                    if not body.startswith("records="):
                        raw_meta_line = body
                    for tok in body.split():
                        if "=" in tok:
                            k, v = tok.split("=", 1)
                            meta[k] = _try_int(v)
                continue

            if line.startswith("t_ms,rec"):
                continue  # column header row

            fields = line.split(",")
            if len(fields) != 11:
                errors += 1
                continue
            try:
                t_raw = int(fields[0])
                rec = fields[1]
                sensors = [int(x) for x in fields[2:7]]
                a, b = int(fields[7]), int(fields[8])
                left, right = int(fields[9]), int(fields[10])
            except ValueError:
                errors += 1
                continue

            # t_ms is uint16 and wraps at 65536 — unwrap monotonically.
            if last_raw is not None and t_raw < last_raw:
                offset += 65536
            last_raw = t_raw
            t_ms = t_raw + offset

            if len(rec) == 1 and rec in TICK_LETTERS:
                ticks.append({"t": t_ms, "rec": rec, "s": sensors,
                              "a": a, "b": b, "left": left, "right": right})
            elif rec:
                events.append({"t": t_ms, "name": rec, "a": a, "b": b})
            else:
                errors += 1

    return meta, ticks, events, raw_meta_line, errors, complete


def _cal_spans(meta):
    """Per-sensor calibration spans from the dump-facts line's cal_min=/
    cal_max= comma-lists. None when the dump predates those tokens or the
    lists don't parse — absence is not an error, just an older CSV."""
    lo, hi = meta.get("cal_min"), meta.get("cal_max")
    if lo is None or hi is None:
        return None
    try:
        lo = [int(x) for x in str(lo).split(",")]
        hi = [int(x) for x in str(hi).split(",")]
    except ValueError:
        return None
    if len(lo) != 5 or len(hi) != 5:
        return None
    return [h - l for l, h in zip(lo, hi)]


def print_summary(meta, ticks, events, raw_meta_line, errors):
    print("=== meta ===")
    print(raw_meta_line or " ".join(f"{k}={v}" for k, v in meta.items()))
    if "records" in meta:
        print(f"records claimed by header: {meta['records']}  "
              f"ring wrapped: {'yes' if meta.get('wrapped') else 'no'}")
    # raw_meta_line is the KNOB line (header line 2); the dump-facts line
    # (line 3) hides behind the records= filter above, so its tokens are
    # surfaced here explicitly or the reader never sees them.
    if isinstance(meta.get("batt_mv"), int):
        mv = meta["batt_mv"]
        print(f"battery at dump: {mv} mV ({mv / 1000:.2f} V, resting — "
              f"loaded voltage during the run was lower)")
    spans = _cal_spans(meta)
    if spans is not None:
        bar = meta.get("cal_min_span")
        verdict = ""
        if isinstance(bar, int):
            bad = [f"s{i}" for i, s in enumerate(spans) if s < bar]
            verdict = (f"  ** {','.join(bad)} BELOW cal_min_span={bar} **"
                       if bad else f"  (all >= cal_min_span={bar})")
        print("cal spans at dump: " + " ".join(str(s) for s in spans) + verdict)

    print("\n=== ticks by phase ===")
    counts = Counter(t["rec"] for t in ticks)
    for letter in "FBCTKJA":
        if counts.get(letter):
            print(f"  {letter} ({PHASE_NAME[letter]:<11}): {counts[letter]}")
    print(f"  total: {len(ticks)}")

    if ticks:
        t0, t1 = ticks[0]["t"], ticks[-1]["t"]
        span = t1 - t0
        print(f"\n=== wall-clock span ===\n  {span} ms ({span / 1000:.2f} s), "
              f"first tick t={t0} ms, last tick t={t1} ms")

    follow = [t for t in ticks if t["rec"] == "F"]
    if follow:
        print("\n=== line-follow (F) ===")
        # "Saturated" means the PID steer asked for more correction than the
        # base speed can absorb (one wheel already at base, the other pinned
        # at zero) — so the threshold is THE RUN'S OWN base speed, and every
        # run announces its base in RUN_START.b. Judging a 3500-base replay
        # lap against the meta line's explore speed manufactures ~100%
        # "saturation" out of healthy steering and sends the tuner at the
        # wrong knob. The dump's own structure gives the honest grouping:
        # each RUN_START opens a run; its F ticks are judged against its b.
        starts = [ev for ev in events if ev["name"] == "RUN_START"]
        segments = []   # (label, base or None, base_src, [F ticks])
        if starts:
            pre = [t for t in follow if t["t"] < starts[0]["t"]]
            if pre:
                # The ring overwrites oldest-first: a long run can scroll
                # its own RUN_START away. These ticks' base is unknown.
                segments.append(("pre-RUN_START (scrolled off the ring)",
                                 None, "", pre))
            for i, ev in enumerate(starts):
                t_end = starts[i + 1]["t"] if i + 1 < len(starts) else None
                seg = [t for t in follow
                       if t["t"] >= ev["t"] and (t_end is None or t["t"] < t_end)]
                mode = (MODE_NAMES[ev["a"]] if 0 <= ev["a"] < len(MODE_NAMES)
                        else f"mode{ev['a']}")
                segments.append((f"run {i + 1} ({mode})", ev["b"],
                                 "RUN_START.b", seg))
        else:
            segments.append(("no RUN_START in ring", None, "", follow))
        for label, base, base_src, seg in segments:
            if not seg:
                continue
            abs_p = [abs(t["a"]) for t in seg]
            print(f"  {label}: {len(seg)} ticks   "
                  f"mean|p| = {sum(abs_p) / len(abs_p):.1f}   "
                  f"max|p| = {max(abs_p)}")
            if base is None:
                # Base unknown — fall back to the meta line's explore speed,
                # and SAY so: a threshold substituted silently is a lie the
                # reader has no way to catch.
                base = meta.get("explore")
                base_src = "meta explore= — fallback, run's own base unknown"
            if isinstance(base, int) and base > 0:
                sat = sum(1 for t in seg if abs(t["b"]) >= base)
                print(f"    steering saturated (|pid| >= base={base}, "
                      f"{base_src}): {100.0 * sat / len(seg):.1f}% of ticks")
            else:
                # No usable threshold: RUN_START.b was 0, or the fallback
                # meta line never carried explore=. Printing nothing would
                # leave the tick line above looking like the whole story
                # and read as "no saturation" — the same silent judgement,
                # just quieter. Say the stat is missing and why, so the
                # reader knows to go get the base.
                print(f"    steering saturation: no stat — this run's base "
                      f"reads {base!r}"
                      + (f" ({base_src})" if base_src else "")
                      + ", so there is no honest threshold to judge "
                        "these ticks against")

    print(f"\n=== events ({len(events)}) ===")
    print(f"  {'t(s)':>9}  {'name':<12} {'a':>7} {'b':>7}")
    for ev in events:
        extra = ""
        if ev["name"] == "CLASSIFY" and 0 <= ev["a"] < len(JCT_NAMES):
            extra = f"  [{JCT_NAMES[ev['a']]}]"
        elif ev["name"] == "RUN_START" and 0 <= ev["a"] < len(MODE_NAMES):
            extra = f"  [mode={MODE_NAMES[ev['a']]}, base={ev['b']}]"
        elif ev["name"] in ("ABORT", "TIMEOUT") and ev["a"] in TICK_CODE_LETTER:
            letter = TICK_CODE_LETTER[ev["a"]]
            extra = f"  [phase={letter}/{PHASE_NAME[letter]}]"
        print(f"  {ev['t'] / 1000:9.3f}  {ev['name']:<12} {ev['a']:7d} {ev['b']:7d}{extra}")

    print(f"\nparse errors (skipped lines): {errors}")


def _phase_spans(ticks, ts):
    """Contiguous [start_s, end_s, letter] runs of non-F tick phases.

    Snapshot ticks (J/A) are invisible here: a snapshot is one instant
    inside whatever phase surrounds it. Letting one open a "span" would
    shade a stretch of time the robot never spent in any phase — a
    one-row lie painted across all three panels."""
    spans, cur = [], None
    for t in ticks:
        letter = t["rec"]
        if letter in SNAPSHOT_LETTERS:
            continue
        x = ts(t["t"])
        if letter != "F":
            if cur is not None and cur[2] == letter:
                cur[1] = x
            else:
                if cur is not None:
                    spans.append(cur)
                cur = [x, x, letter]
        elif cur is not None:
            spans.append(cur)
            cur = None
    if cur is not None:
        spans.append(cur)
    return spans


def plot_run(meta, ticks, events, out_path, title_name):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    if not ticks:
        print("no tick rows to plot — nothing to draw")
        return

    t0 = ticks[0]["t"]
    def ts(ms):
        return (ms - t0) / 1000.0

    fig, (ax_sens, ax_err, ax_duty) = plt.subplots(
        3, 1, sharex=True, figsize=(16, 9), dpi=130)
    xs_all = [ts(t["t"]) for t in ticks]

    # ---- background shading for non-F phases, all three axes ----
    spans = _phase_spans(ticks, ts)
    for ax in (ax_sens, ax_err, ax_duty):
        for xs, xe, letter in spans:
            ax.axvspan(xs, max(xe, xs + 0.001), color=PHASE_COLOR[letter],
                       alpha=0.15, lw=0, zorder=0)
    # Spans can only ever hold B/C/T/K — _phase_spans skips F and the
    # J/A snapshots — so this key covers every letter that can appear.
    seen_letters = sorted({s[2] for s in spans}, key="BCTK".index)
    phase_patches = [mpatches.Patch(color=PHASE_COLOR[l], alpha=0.3,
                                     label=f"{l} {PHASE_NAME[l]}")
                      for l in seen_letters]

    # ---- panel 1: sensors ----
    for i in range(5):
        ax_sens.plot(xs_all, [t["s"][i] for t in ticks],
                     color=SENSOR_COLORS[i], lw=1.2, label=f"s{i}")
    outer_thresh, lost_thresh = meta.get("outer_thresh"), meta.get("lost_thresh")
    if isinstance(outer_thresh, int):
        ax_sens.axhline(outer_thresh, color=REF_COLOR, ls="--", lw=1,
                        label=f"outer_thresh={outer_thresh}")
    if isinstance(lost_thresh, int):
        ax_sens.axhline(lost_thresh, color=REF_COLOR, ls="--", lw=1,
                        label=f"lost_thresh={lost_thresh}")
    ax_sens.set_ylabel("line sensors\n(0=white .. 1000=black, s0=leftmost)")
    ax_sens.set_ylim(-30, 1030)
    ax_sens.legend(loc="upper right", ncol=7, fontsize=7.5, framealpha=0.85)

    # ---- panel 2: position error (F/B only — a means something else
    # on C/T/K ticks) ----
    fb = [t for t in ticks if t["rec"] in ("F", "B")]
    if fb:
        ax_err.plot([ts(t["t"]) for t in fb], [t["a"] for t in fb],
                    color="#2a78d6", lw=1.2, label="p (position error, F/B)")
    deadend_p_max = meta.get("deadend_p_max")
    if isinstance(deadend_p_max, int):
        ax_err.axhline(deadend_p_max, color=REF_COLOR, ls="--", lw=1,
                       label=f"±deadend_p_max={deadend_p_max}")
        ax_err.axhline(-deadend_p_max, color=REF_COLOR, ls="--", lw=1)
    ax_err.set_ylabel("position error p\n(-2000 left .. +2000 right)")
    ax_err.legend(loc="upper right", fontsize=7.5, framealpha=0.85)

    # ---- events: thin line on all three axes, label on this (err) axis ----
    ylim = ax_err.get_ylim()
    for ev in events:
        x = ts(ev["t"])
        for ax in (ax_sens, ax_err, ax_duty):
            ax.axvline(x, color=EVENT_COLOR, lw=0.6, alpha=0.55, zorder=1)
        ax_err.text(x, ylim[1], ev["name"], rotation=90, fontsize=6.5,
                    va="top", ha="center", color=EVENT_COLOR)

    # ---- panel 3: commanded duty ----
    ax_duty.plot(xs_all, [t["left"] for t in ticks],
                color=DUTY_COLORS[0], lw=1.2, label="left duty")
    ax_duty.plot(xs_all, [t["right"] for t in ticks],
                color=DUTY_COLORS[1], lw=1.2, label="right duty")
    # Label the axis for what these rows ARE: the command as computed,
    # logged before hw_motors.c clamps it. The ±5000 hard cap bounds the
    # EXECUTED duty, so a trace can legitimately run past the cap here.
    ax_duty.set_ylabel("commanded duty, pre-clamp\n(executed duty capped at ±5000)")
    ax_duty.set_xlabel("time (s)")
    ax_duty.legend(loc="upper right", fontsize=7.5, framealpha=0.85)

    for ax in (ax_sens, ax_err, ax_duty):
        ax.grid(True, color="#e1e0d9", lw=0.6)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    if phase_patches:
        fig.legend(handles=phase_patches, loc="upper center",
                   ncol=len(phase_patches), bbox_to_anchor=(0.5, 0.955),
                   fontsize=8, framealpha=0.85, title="shaded phase")

    kp, kd, period = meta.get("kp"), meta.get("kd"), meta.get("period_us")
    fig.suptitle(f"{title_name}   kp={kp} kd={kd} period_us={period}",
                fontsize=12, y=0.998)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(out_path, dpi=130)
    print(out_path)


def main():
    ap = argparse.ArgumentParser(description="Plot a 3pi2040 telemetry dump.")
    ap.add_argument("csv", help="telemetry CSV from LOG DUMP")
    ap.add_argument("--out", help="output PNG path (default: <csv>.png)")
    ap.add_argument("--summary", action="store_true",
                     help="print stats to stdout in addition to plotting")
    args = ap.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        sys.exit(f"error: {csv_path} not found")

    meta, ticks, events, raw_meta_line, errors, complete = parse_csv(csv_path)

    if not complete:
        # The dump prints '# end' last, always — no terminator means the
        # capture was cut off, and the run's TAIL (usually the interesting
        # part: the fault, the goal) may be missing. Warn loudly but still
        # plot: a truncated dump is evidence that must say so, not garbage.
        print("WARNING: no '# end' terminator — capture was cut off; "
              "the tail of the run may be missing from everything below.")

    if args.summary:
        print_summary(meta, ticks, events, raw_meta_line, errors)

    out_path = args.out or str(csv_path.with_suffix(".png"))
    try:
        plot_run(meta, ticks, events, out_path, csv_path.name)
    except ImportError:
        # Homebrew's Python refuses a bare pip install (PEP 668: the
        # interpreter belongs to Homebrew), so the venv below is the
        # supported route, not a detour.
        print("matplotlib not installed — one-time setup, from the repo root:\n"
              "  python3 -m venv .venv\n"
              "  .venv/bin/pip install matplotlib\n"
              "then plot with the venv's interpreter:\n"
              "  .venv/bin/python3 tools/plot_telemetry.py <csv> --summary\n"
              "(--summary output above still works without it.)")


if __name__ == "__main__":
    main()
