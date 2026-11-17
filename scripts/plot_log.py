#!/usr/bin/env python3
"""Plot a sample capture from `pio device monitor`.

The firmware prints a CSV header, either raw counts
("t_us,pos,adc_raw,ads_code,ads_new") or volts ("t_us,pos,adc_v,ads_v,ads_new"),
then one row per sample. Comment lines ("# dropped N") are counted, and
anything else that is not a numeric row (boot noise, fault messages) is skipped.

Captures that started partway through a run have no header. The layout is then
inferred from the rows themselves, and the first row is dropped in case the
capture cut into it. If the board reset during a capture, each run (each header)
is plotted as its own figure, since micros() restarts at every boot.

A second window fits the sensor dips against stepper position to locate the
metal target, and a comparison of the two ADCs' repeatability and accuracy is
printed to the console (see dip_analysis.py).

    python scripts/plot_log.py                  # newest log in logs/
    python scripts/plot_log.py logs/foo.log     # specific log(s); globs OK
    python scripts/plot_log.py --true-pos 120   # known target position [steps]
    python scripts/plot_log.py --window 5 10    # load cell: residual over 5-10 s only
"""

import argparse
import glob
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import dip_analysis

LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
HEADER_PREFIX = "t_us,"

# Rows used to infer a headerless layout.
INFER_SAMPLE_ROWS = 50


def infer_names(rows):
    """Guess the column names of headerless rows (lists of field strings).

    Field count separates the layouts, and within one count, whether a column
    is printed with a decimal point separates counts from volts.

    Returns (names, field_count), or (None, None) if no known layout fits.
    """
    if not rows:
        return None, None
    count = Counter(len(r) for r in rows).most_common(1)[0][0]
    sample = [r for r in rows if len(r) == count][:INFER_SAMPLE_ROWS]

    def dotted(i):
        return sum("." in r[i] for r in sample) > len(sample) / 2

    if count == 4:
        # Load-cell firmware (loadcell.cpp).
        return ["t_us", "ads_code", "uv", "mv_per_v"], count
    if count == 5:
        if dotted(2):
            return ["t_us", "pos", "adc_v", "ads_v", "ads_new"], count
        return ["t_us", "pos", "adc_raw", "ads_code", "ads_new"], count
    if count == 6:
        # Earlier firmware: a duty column in place of pos, then volts + code.
        second = "duty" if dotted(1) else "pos"
        return ["t_us", second, "adc_v", "ads_code", "ads_v", "ads_new"], count
    return None, None


def numeric_fields(line):
    """Split a CSV line into fields if every one is a number, else None."""
    fields = line.split(",")
    try:
        for f in fields:
            float(f)
    except ValueError:
        return None
    return fields


class Segment:
    """One boot's worth of rows: a header (or inferred names) and its data."""

    def __init__(self, names):
        self.names = names
        self.rows = []
        self.dropped = 0
        self.inferred = names is None


def load(path):
    """Split a log into Segments, one per run. Never raises on bad content."""
    text = path.read_text(errors="replace")
    lines = text.splitlines()
    # The capture can stop mid-line; a truncated last row may still have the
    # right field count with a cut-off number in it, so drop it.
    if lines and not text.endswith(("\n", "\r")):
        lines = lines[:-1]

    segments = [Segment(None)]
    for raw in lines:
        line = raw.strip()
        if not line:
            continue
        if line.startswith(HEADER_PREFIX):
            segments.append(Segment(line.split(",")))
            continue
        if line.startswith("# dropped"):
            try:
                segments[-1].dropped += int(line.split()[-1])
            except ValueError:
                pass
            continue
        fields = numeric_fields(line)
        if fields is not None:
            segments[-1].rows.append(fields)

    # The leading headerless segment holds rows from before the first header:
    # a capture that started partway. Infer its layout, and drop its first row,
    # which the capture may have cut into.
    head = segments[0]
    if head.rows:
        head.rows = head.rows[1:]
        head.names, _ = infer_names(head.rows)
        if head.names is None:
            print(f"{path}: {len(head.rows)} rows before the header have an unknown layout; skipped")
            head.rows = []

    usable = []
    for seg in segments:
        width = len(seg.names) if seg.names else 0
        rows = [[float(f) for f in r] for r in seg.rows if len(r) == width]
        if not rows:
            continue
        data = np.array(rows, dtype=float)
        # A reset whose header never made it into the capture still shows as
        # micros() jumping backwards; split there too, so each run keeps its own
        # time axis.
        starts = [0] + [i + 1 for i in resets(data[:, 0])] + [len(data)]
        for k, (a, b) in enumerate(zip(starts, starts[1:])):
            part = seg if k == 0 else Segment(seg.names)
            part.inferred = seg.inferred
            part.cols = {name: data[a:b, i] for i, name in enumerate(seg.names)}
            usable.append(part)
    return usable


# micros() wraps from 2^32 - 1 to 0 every ~71.6 min. A backwards jump that
# starts this close to the top and lands this close to 0 is a wrap, not a reset.
WRAP_MARGIN_US = 100_000


def resets(t_us):
    """Indices i where t_us[i + 1] < t_us[i] because the board restarted."""
    back = np.nonzero(np.diff(t_us) < 0)[0]
    wrap = (t_us[back] > 2**32 - WRAP_MARGIN_US) & (t_us[back + 1] < WRAP_MARGIN_US)
    return back[~wrap]


def break_gaps(t, y):
    """Insert NaN where rows are missing, so plots leave a gap instead of a line."""
    if len(t) < 3:
        return t, y
    gap = np.nonzero(np.diff(t) > 3 * np.median(np.diff(t)))[0] + 1
    return np.insert(t, gap, np.nan), np.insert(y.astype(float), gap, np.nan)


# Load-cell full scale: +/-AVDD/128 at the input, +/-1000/128 mV/V.
LOADCELL_FS_MVV = 1000.0 / 128.0


def plot_loadcell(title, seg, window=None):
    """Load-cell capture: mV/V against time, with noise figures printed.

    window, if given, is (start_s, end_s) on the plot's time axis: the noise
    figures and residuals are then computed only over that span, e.g. the part
    of the capture where a constant load was on the cell.
    """
    cols = seg.cols
    t_us = np.unwrap(cols["t_us"], period=2**32)
    t = (t_us - t_us[0]) * 1e-6
    mvv_all = cols["mv_per_v"]
    fs = 1.0 / np.median(np.diff(t)) if len(t) > 1 else float("nan")

    if window is not None:
        lo, hi = window
        sel = (t >= lo) & (t <= hi)
        if np.count_nonzero(sel) < 3:
            print(f"\n=== {title}: load cell ===")
            print(f"window {lo:g}-{hi:g} s holds {np.count_nonzero(sel)} samples "
                  f"(capture runs 0-{t[-1]:.1f} s); need at least 3")
            return
        span_note = f"{lo:g}-{hi:g} s"
    else:
        sel = np.ones(len(t), dtype=bool)
        span_note = "whole capture"
    tw, mvv, uv = t[sel], mvv_all[sel], cols["uv"][sel]

    # Residuals: deviation from the window's mean, and from a straight-line fit
    # (which takes out slow drift such as creep or warm-up).
    resid_uvv = (mvv - np.mean(mvv)) * 1e3
    slope, intercept = np.polyfit(tw, mvv, 1)
    detr_uvv = (mvv - (slope * tw + intercept)) * 1e3

    rms_uv = np.std(uv)
    pp_uv = np.ptp(uv)
    rms_mvv = np.std(mvv)
    span = 2 * LOADCELL_FS_MVV  # full range in mV/V
    enob = np.log2(span / (rms_mvv * np.sqrt(12))) if rms_mvv > 0 else float("inf")
    noise_free = np.log2(span / np.ptp(mvv)) if np.ptp(mvv) > 0 else float("inf")
    print(f"\n=== {title}: load cell ({span_note}) ===")
    print(f"{len(tw)} samples over {tw[-1] - tw[0]:.1f} s ({fs:.2f} SPS)")
    print(f"mean   {np.mean(mvv):+.6f} mV/V   ({np.mean(uv):+.2f} uV)")
    print(f"noise  {rms_uv:.3f} uV rms, {pp_uv:.3f} uV p-p   ({rms_mvv * 1e3:.4f} uV/V rms)")
    print(f"       ~{enob:.1f} effective bits, ~{noise_free:.1f} noise-free bits "
          f"over the +/-{LOADCELL_FS_MVV:.2f} mV/V range")
    print(f"drift  {slope * 1e3:+.4f} uV/V per s (linear fit)")
    print(f"resid  {np.std(resid_uvv):.4f} uV/V rms about the mean, "
          f"{np.std(detr_uvv):.4f} uV/V rms about the linear fit "
          f"({np.ptp(detr_uvv):.4f} p-p)")
    if window is None:
        print("These are only noise figures while the load is constant; if you loaded the "
              "cell during the capture, they include that change (use --window).")

    fig, axes = plt.subplots(3, 1, figsize=(10, 9))
    fig.suptitle(title)
    axes[0].plot(*break_gaps(t, mvv_all))
    if window is not None:
        axes[0].axvspan(tw[0], tw[-1], color="tab:orange", alpha=0.15,
                        label=f"window {span_note}")
        axes[0].plot(tw, slope * tw + intercept, color="tab:orange", lw=1,
                     label="linear fit")
        axes[0].legend(fontsize=8, loc="best")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Bridge output (mV/V)")
    axes[0].set_title(f"{len(tw)} samples ({span_note})  {fs:.1f} SPS  "
                      f"noise {rms_uv:.2f} uV rms", fontsize=9, loc="right")

    axes[1].plot(*break_gaps(tw, resid_uvv), lw=0.8, label="about mean")
    axes[1].plot(*break_gaps(tw, detr_uvv), lw=0.8, label="about linear fit")
    axes[1].axhline(0, color="k", lw=0.5)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Residual (uV/V)")
    axes[1].legend(fontsize=8, loc="best")

    axes[2].hist(resid_uvv, bins=50, alpha=0.6, label="about mean")
    axes[2].hist(detr_uvv, bins=50, alpha=0.6, label="about linear fit")
    axes[2].set_xlabel(f"Residual (uV/V), {span_note}")
    axes[2].set_ylabel("Samples")
    axes[2].legend(fontsize=8, loc="best")
    for ax in axes:
        ax.grid(True)
    fig.tight_layout()


def plot_segment(title, seg):
    cols = seg.cols
    # micros() wraps every ~71.6 min; unwrap so long captures stay monotonic.
    t_us = np.unwrap(cols["t_us"], period=2**32)
    t = (t_us - t_us[0]) * 1e-6
    fs = 1.0 / np.median(np.diff(t)) if len(t) > 1 else float("nan")

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig.suptitle(title)
    stale = int(np.sum(cols["ads_new"] == 0)) if "ads_new" in cols else 0
    note = "  (no header: layout inferred)" if seg.inferred else ""
    axes[0].set_title(
        f"{len(t)} samples  fs={fs:.1f} Hz  dropped={seg.dropped}  "
        f"stale ADS rows={stale}{note}",
        fontsize=9,
        loc="right",
    )

    if "ads_v" in cols:
        axes[0].plot(*break_gaps(t, cols["ads_v"]))
        axes[0].set_ylabel("ADS1220 (V)")
    elif "ads_code" in cols:
        axes[0].plot(*break_gaps(t, cols["ads_code"]))
        axes[0].set_ylabel("ADS1220 (counts)")

    if "adc_v" in cols:
        axes[1].plot(*break_gaps(t, cols["adc_v"]))
        axes[1].set_ylabel("STM32 ADC (V)")
    elif "adc_raw" in cols:
        axes[1].plot(*break_gaps(t, cols["adc_raw"]))
        axes[1].set_ylabel("STM32 ADC (counts)")

    if "pos" in cols:
        axes[2].step(*break_gaps(t, cols["pos"]), where="post")
        axes[2].set_ylabel("Stepper (steps)")
    elif "duty" in cols:
        axes[2].plot(*break_gaps(t, cols["duty"]))
        axes[2].set_ylabel("Motor duty")
    axes[2].set_xlabel("Time (s)")
    for ax in axes:
        ax.grid(True)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("logs", nargs="*")
    parser.add_argument(
        "--window", nargs=2, type=float, metavar=("START", "END"),
        help="load-cell logs: only compute noise/residuals between START and END "
             "seconds (plot time axis), e.g. --window 5 10 for a constant-load span",
    )
    dip_analysis.add_arguments(parser)
    args = parser.parse_args()
    if args.window and args.window[1] <= args.window[0]:
        parser.error("--window END must be greater than START")

    # windows doesn't expand wildcards
    logs = [Path(f) for p in args.logs for f in glob.glob(p)]
    if args.logs and not logs:
        raise SystemExit(f"no files match {' '.join(args.logs)}")
    if not logs:
        found = sorted(LOG_DIR.glob("*.log"), key=lambda p: p.stat().st_mtime)
        if not found:
            raise SystemExit(f"no logs given and none in {LOG_DIR}")
        logs = [found[-1]]

    plotted = 0
    for path in logs:
        segments = load(path)
        if not segments:
            print(f"{path}: no sample rows found; skipped")
            continue
        for i, seg in enumerate(segments):
            title = path.stem if len(segments) == 1 else f"{path.stem} (run {i + 1})"
            if "mv_per_v" in seg.cols:
                plot_loadcell(title, seg, args.window)
                plotted += 1
                continue
            plot_segment(title, seg)
            if not args.no_dips:
                dip_analysis.analyse(title, seg.cols, args)
            plotted += 1

    if plotted:
        plt.show()


if __name__ == "__main__":
    main()
