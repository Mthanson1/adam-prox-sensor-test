"""Locate the metal target from inductive-sensor dips and compare the two ADCs.

The sensor reads ~10.7 V with no metal in range and falls toward 0 V as metal
approaches. While the stepper sweeps the target past it, each pass of the
target shows up as a dip in sensor voltage vs stepper position. For each sweep
(one move from one end to the other) and each ADC:

  1. Dips are the stretches below half depth: halfway between the no-metal
     baseline and the deepest reading in the capture.
  2. A parabola is fitted to the bottom FIT_FRAC of each dip (voltage vs
     position). Its vertex is the position estimate for that pass, with
     sub-step resolution. A clipped flat bottom still gives a centred vertex.
  3. Dips are grouped by position across sweeps (CLUSTER_GAP_STEPS), so a
     target with several metal features gives one group per feature.

Statistics are kept per group and per sweep direction: sensor lag and backlash
shift forward and reverse passes in opposite directions, so pooling them would
inflate the scatter.

Accuracy needs ground truth this log does not have. Without it the ADS1220
(24-bit) is taken as the reference and the STM32 ADC is scored against it; pass
--true-pos to score both against the known position instead.
"""

from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np

# Conversion defaults, matching the firmware (RigConfig.h, AdcToVolt.h).
DIVIDER_GAIN = (2231.0 + 1000.0) / 1000.0  # sensor V per ADC-input V
STM_VREF_V = 3.3
STM_FULL_SCALE = 4095.0
ADS_VREF_V = 5.0  # AVDD reference (Nucleo 5V rail)
ADS_FULL_SCALE = float(2**23)

# Dip detection and fitting.
FIT_FRAC = 0.3  # fit the bottom 30% of each dip's depth
MIN_DEPTH_V = 0.5  # captures with less swing than this have no dips
CLUSTER_GAP_STEPS = 20.0  # dips further apart than this are different features
MIN_FIT_POSITIONS = 3  # distinct step positions needed for a parabola

# Stationary-noise windows: dwells at least this long, skipping the start.
MIN_DWELL_S = 0.2
DWELL_SETTLE_S = 0.05


@dataclass
class Channel:
    """One ADC's sensor-voltage trace."""

    name: str
    volts: np.ndarray
    lsb_v: float  # sensor volts per count
    valid: np.ndarray  # rows holding a fresh reading
    full_scale_counts: float

    @property
    def full_scale_v(self):
        return self.lsb_v * self.full_scale_counts

    @property
    def clipped(self):
        """Rows pinned at either end of the ADC range, where noise is invisible."""
        return (self.volts >= self.full_scale_v - 1.5 * self.lsb_v) | (self.volts <= 0.5 * self.lsb_v)


@dataclass
class Dip:
    channel: str
    pass_index: int
    direction: int  # +1 increasing position, -1 decreasing
    position: float  # fitted vertex [steps]
    v_min: float  # fitted vertex voltage [V]
    fit_pos: np.ndarray = field(repr=False)
    coeffs: np.ndarray = field(repr=False)
    group: int = -1


def channels_from(cols, args):
    """Sensor-voltage traces for whichever ADC columns the log has."""
    gain = args.gain
    stm_lsb = args.stm_vref / STM_FULL_SCALE * gain
    ads_lsb = args.ads_vref / ADS_FULL_SCALE * gain
    n = len(cols["t_us"])
    fresh = cols["ads_new"] != 0 if "ads_new" in cols else np.ones(n, bool)
    out = []
    if "ads_code" in cols:
        out.append(Channel("ADS1220", cols["ads_code"] * ads_lsb, ads_lsb, fresh, ADS_FULL_SCALE - 1))
    elif "ads_v" in cols:
        out.append(Channel("ADS1220", cols["ads_v"], ads_lsb, fresh, ADS_FULL_SCALE - 1))
    all_rows = np.ones(n, bool)
    if "adc_raw" in cols:
        out.append(Channel("STM32", cols["adc_raw"] * stm_lsb, stm_lsb, all_rows, STM_FULL_SCALE))
    elif "adc_v" in cols:
        out.append(Channel("STM32", cols["adc_v"], stm_lsb, all_rows, STM_FULL_SCALE))
    return out


def find_passes(pos, t, fs):
    """(start, stop, direction) index ranges of each sweep.

    A sweep is a run of position changes in one direction with no pause longer
    than 0.1 s between steps. A gap in the log (rows missing, e.g. the firmware
    logging only positive moves) also ends a sweep, and a position jump across
    such a gap is not counted as motion.
    """
    gap = np.diff(t) > 3.0 / fs
    moved = (np.diff(pos) != 0) & ~gap
    changes = np.nonzero(moved)[0]
    if len(changes) == 0:
        return []
    gaps_before = np.concatenate(([0], np.cumsum(gap)))  # gaps up to each row
    signs = np.sign(np.diff(pos)[changes])
    passes = []
    start = 0
    for k in range(1, len(changes) + 1):
        end_of_run = (
            k == len(changes)
            or signs[k] != signs[start]
            or t[changes[k]] - t[changes[k - 1]] > 0.1
            or gaps_before[changes[k]] != gaps_before[changes[k - 1]]
        )
        if end_of_run:
            i0, i1 = changes[start], changes[k - 1] + 1
            if abs(pos[i1] - pos[i0]) >= 2:
                passes.append((i0, i1 + 1, int(signs[start])))
            start = k
    return passes


def fit_dip(p, v, baseline):
    """Parabola through the bottom of one dip. Returns (vertex, v_min, mask, coeffs) or None."""
    local_min = v.min()
    keep = v <= local_min + FIT_FRAC * (baseline - local_min)
    if len(np.unique(p[keep])) < MIN_FIT_POSITIONS:
        return None
    coeffs = np.polyfit(p[keep], v[keep], 2)
    a, b, c = coeffs
    if a <= 0:
        return None  # opens downward: not a dip
    vertex = -b / (2 * a)
    if not (p[keep].min() <= vertex <= p[keep].max()):
        return None
    return vertex, c - b * b / (4 * a), keep, coeffs


def find_dips(channel, pos, passes):
    v_all = channel.volts[channel.valid]
    if len(v_all) == 0:
        return []
    baseline = np.percentile(v_all, 95)
    floor = np.percentile(v_all, 0.5)
    if baseline - floor < MIN_DEPTH_V:
        return []
    threshold = floor + 0.5 * (baseline - floor)

    dips = []
    for k, (i0, i1, direction) in enumerate(passes):
        idx = np.arange(i0, i1)
        idx = idx[channel.valid[idx]]
        p, v = pos[idx], channel.volts[idx]
        below = v < threshold
        # Contiguous below-threshold runs, skipping any cut off by the pass ends.
        edges = np.diff(np.concatenate(([0], below.astype(int), [0])))
        for r0, r1 in zip(np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0]):
            if r0 == 0 or r1 == len(v):
                continue
            fit = fit_dip(p[r0:r1], v[r0:r1], baseline)
            if fit is None:
                continue
            vertex, v_min, keep, coeffs = fit
            dips.append(Dip(channel.name, k, direction, vertex, v_min, p[r0:r1][keep], coeffs))
    return dips


def group_dips(dips):
    """Assign dips to position groups (metal features), shared across channels."""
    if not dips:
        return 0
    order = sorted(dips, key=lambda d: d.position)
    g = 0
    order[0].group = 0
    for prev, cur in zip(order, order[1:]):
        if cur.position - prev.position > CLUSTER_GAP_STEPS:
            g += 1
        cur.group = g
    return g + 1


def stationary_noise(channel, pos, fs, min_dwell_s=MIN_DWELL_S, settle_s=DWELL_SETTLE_S):
    """Pooled RMS noise [V] over dwells, ignoring clipped readings.

    Each dwell's variance uses ddof=1, so short dwells (a sweep that pauses only
    a few samples per step) are not biased low by removing their own mean.

    Returns (rms, dwells used, dwells skipped because they were clipped).
    """
    min_len = int(round(min_dwell_s * fs))
    settle = int(round(settle_s * fs))
    # Need a few samples left after settling to estimate a variance at all.
    min_good = max(3, min(10, min_len - settle))
    change = np.concatenate(([True], np.diff(pos) != 0, [True]))
    bounds = np.nonzero(change)[0]
    usable = channel.valid & ~channel.clipped
    variances, counts, clipped_dwells = [], [], 0
    for s, e in zip(bounds[:-1], bounds[1:]):
        if e - s < min_len:
            continue
        idx = np.arange(s + settle, e)
        good = idx[usable[idx]]
        if len(good) < 0.5 * len(idx) or len(good) < min_good:
            clipped_dwells += 1
            continue
        variances.append(np.var(channel.volts[good], ddof=1))
        counts.append(len(good) - 1)  # weight by degrees of freedom
    if not counts:
        return float("nan"), 0, clipped_dwells
    return float(np.sqrt(np.average(variances, weights=counts))), len(counts), clipped_dwells


def fmt_pos(steps, args):
    """A position or spread [steps] in steps, motor degrees and optionally mm."""
    s = f"{steps:9.3f} st  {steps * 360.0 / args.steps_per_rev:8.3f} deg"
    if args.mm_per_rev:
        s += f"  {steps * args.mm_per_rev / args.steps_per_rev:8.4f} mm"
    return s


def report(title, channels, dips, n_groups, pos, fs, args):
    """Print the numerical repeatability / accuracy comparison."""
    names = [c.name for c in channels]
    print(f"\n=== {title}: ADC comparison ===")
    print(f"conversion: divider x{args.gain:.4f}, STM32 Vref {args.stm_vref} V, "
          f"ADS1220 Vref {args.ads_vref} V")

    print("\n-- Clipping --")
    for c in channels:
        frac = np.mean(c.clipped[c.valid]) * 100
        print(f"{c.name:8s} {frac:5.1f} % of readings at the end of the ADC range "
              f"(full scale = {c.full_scale_v:.3f} V at the sensor)")

    print("\n-- Stationary noise (repeatability of a single reading) --")
    for c in channels:
        rms, n, n_clipped = stationary_noise(c, pos, fs, args.min_dwell, args.dwell_settle)
        if n == 0:
            why = (f"all {n_clipped} dwells clipped (sensor above full scale)" if n_clipped
                   else f"no dwells of {args.min_dwell} s or longer (try --min-dwell)")
            print(f"{c.name:8s} not measurable: {why}")
            continue
        enob = np.log2(c.full_scale_v / (rms * np.sqrt(12))) if rms > 0 else float("inf")
        extra = f", {n_clipped} clipped dwells skipped" if n_clipped else ""
        print(f"{c.name:8s} {rms * 1e3:9.3f} mV rms  {rms / c.lsb_v:10.2f} counts rms  "
              f"~{enob:5.1f} effective bits  ({n} dwells{extra})")

    if len(channels) == 2:
        ref, dut = channels
        both = ref.valid & dut.valid & ~ref.clipped & ~dut.clipped
        if both.sum() > 10:
            g, o = np.polyfit(ref.volts[both], dut.volts[both], 1)
            resid = dut.volts[both] - (g * ref.volts[both] + o)
            diff = dut.volts[both] - ref.volts[both]
            print(f"\n-- Voltage agreement, {dut.name} vs {ref.name} (reference), unclipped rows --")
            print(f"mean difference {np.mean(diff) * 1e3:+9.3f} mV, "
                  f"rms difference {np.sqrt(np.mean(diff**2)) * 1e3:8.3f} mV")
            print(f"linear fit: gain error {(g - 1) * 100:+7.3f} %, offset {o * 1e3:+8.3f} mV, "
                  f"residual {np.std(resid) * 1e3:7.3f} mV rms, "
                  f"max {np.max(np.abs(resid)) * 1e3:7.3f} mV")

    if not dips:
        print("\nNo dips found: no sweeps through the target, or swing under "
              f"{MIN_DEPTH_V} V.")
        return

    true_pos = list(args.true_pos or [])
    print("\n-- Target position from dip minima (per sweep) --")
    print("sigma / spread = repeatability of the per-sweep fits; "
          f"reference for accuracy = {'--true-pos' if true_pos else names[0]}")
    for g in range(n_groups):
        truth = true_pos[g] if g < len(true_pos) else None
        print(f"\nfeature {g + 1}" + (f" (true position {truth:g} st)" if truth is not None else ""))
        dir_means = {n: {} for n in names}
        for direction, label in ((+1, "forward"), (-1, "reverse")):
            for n in names:
                ds = [d for d in dips if d.group == g and d.direction == direction and d.channel == n]
                if not ds:
                    continue
                p = np.array([d.position for d in ds])
                vm = np.array([d.v_min for d in ds])
                dir_means[n][direction] = p.mean()
                sd = p.std(ddof=1) if len(p) > 1 else float("nan")
                print(f"  {n:8s} {label}  n={len(p):3d}  mean {fmt_pos(p.mean(), args)}")
                print(f"  {'':8s}          sigma {fmt_pos(sd, args)}   "
                      f"spread {p.max() - p.min():7.3f} st   "
                      f"dip min {vm.mean():.4f} +/- {vm.std():.4f} V")
                if truth is not None:
                    err = p - truth
                    print(f"  {'':8s}          error  bias {err.mean():+8.3f} st   "
                          f"rms {np.sqrt(np.mean(err**2)):7.3f} st")

        # One direction only (e.g. the firmware logging positive moves only):
        # compare the ADCs on that direction. The lag bias stays in, but it is
        # the same for both ADCs and matches how the device is used.
        if not any(len(m) == 2 for m in dir_means.values()):
            for direction, label in ((+1, "forward"), (-1, "reverse")):
                if all(direction in dir_means[n] for n in names) and len(names) == 2:
                    n_ref, n_dut = names
                    diff = dir_means[n_dut][direction] - dir_means[n_ref][direction]
                    print(f"  {n_dut} - {n_ref} ({label} only): {diff:+8.3f} st  "
                          f"(accuracy of {n_dut} with {n_ref} as reference)")
            continue

        # Forward and reverse are biased in opposite directions by sensor lag (and
        # backlash), so their average is the best estimate of the true position.
        print("  lag-cancelled position (average of forward and reverse means):")
        combined = {}
        for n in names:
            m = dir_means[n]
            if len(m) < 2:
                print(f"  {n:8s} needs both directions")
                continue
            combined[n] = (m[+1] + m[-1]) / 2
            line = f"  {n:8s} {fmt_pos(combined[n], args)}   fwd-rev split {m[+1] - m[-1]:+7.3f} st"
            if truth is not None:
                line += f"   error {combined[n] - truth:+8.3f} st"
            print(line)
        if len(combined) == 2:
            (n_ref, m_ref), (n_dut, m_dut) = list(combined.items())
            print(f"  {n_dut} - {n_ref}: {m_dut - m_ref:+8.3f} st  "
                  f"(accuracy of {n_dut} with {n_ref} as reference)")


def plot(title, channels, dips, pos, passes):
    """Second window: sensor V vs position with fits, and per-sweep minima."""
    fig, axes = plt.subplot_mosaic(
        [[c.name for c in channels], ["trend"] * len(channels)], figsize=(12, 8)
    )
    fig.suptitle(f"{title}: dip fits")
    colors = {+1: "tab:blue", -1: "tab:orange"}

    # Zoom the V-vs-position plots onto the dips: the fitted region plus twice
    # its width on each side, so the shoulders of each dip are visible too.
    zoom = None
    if dips:
        lo = min(d.fit_pos.min() for d in dips)
        hi = max(d.fit_pos.max() for d in dips)
        pad = 2 * (hi - lo) + 10
        zoom = (lo - pad, hi + pad)

    for c in channels:
        ax = axes[c.name]
        labelled = set()
        for i0, i1, direction in passes:
            idx = np.arange(i0, i1)
            idx = idx[c.valid[idx]]
            label = None
            if direction not in labelled:
                label = "forward" if direction > 0 else "reverse"
                labelled.add(direction)
            ax.plot(pos[idx], c.volts[idx], color=colors[direction], alpha=0.25, lw=0.8,
                    label=label)
        for d in (d for d in dips if d.channel == c.name):
            px = np.linspace(d.fit_pos.min(), d.fit_pos.max(), 50)
            ax.plot(px, np.polyval(d.coeffs, px), color="k", lw=1)
            ax.plot(d.position, d.v_min, "k.", ms=4)
        if zoom:
            ax.set_xlim(*zoom)
        ax.set_title(c.name)
        ax.set_xlabel("Stepper position (steps)")
        ax.set_ylabel("Sensor (V)")
        ax.grid(True)
        ax.legend(loc="lower right", fontsize=8)

    ax = axes["trend"]
    # Hollow circles for the ADS1220 so the STM32 crosses show through when the
    # two land on the same position.
    style = {"ADS1220": dict(marker="o", mfc="none", ms=8), "STM32": dict(marker="x", ms=6)}
    for c in channels:
        for direction in (+1, -1):
            ds = [d for d in dips if d.channel == c.name and d.direction == direction]
            if ds:
                ax.plot([d.pass_index for d in ds], [d.position for d in ds],
                        color=colors[direction], ls="none",
                        label=f"{c.name} {'fwd' if direction > 0 else 'rev'}",
                        **style.get(c.name, dict(marker="s")))
    ax.set_xlabel("Sweep number")
    ax.set_ylabel("Fitted minimum (steps)")
    ax.set_title("Per-sweep target position (repeatability)")
    ax.grid(True)
    ax.legend(fontsize=8, ncol=4, loc="best")
    fig.tight_layout()


def analyse(title, cols, args):
    """Run the dip analysis on one run's columns: print the report, open the window."""
    if "pos" not in cols:
        print(f"{title}: no position column; dip analysis skipped")
        return
    pos = cols["pos"]
    t = np.unwrap(cols["t_us"], period=2**32) * 1e-6
    fs = 1.0 / np.median(np.diff(t)) if len(t) > 1 else 500.0
    channels = channels_from(cols, args)
    passes = find_passes(pos, t, fs)
    dips = [d for c in channels for d in find_dips(c, pos, passes)]
    n_groups = group_dips(dips)
    report(title, channels, dips, n_groups, pos, fs, args)
    if passes:
        plot(title, channels, dips, pos, passes)


def add_arguments(parser):
    g = parser.add_argument_group("dip analysis")
    g.add_argument("--no-dips", action="store_true", help="skip the dip-fit window and report")
    g.add_argument("--gain", type=float, default=DIVIDER_GAIN,
                   help=f"divider gain, sensor V per ADC V (default {DIVIDER_GAIN:.4f})")
    g.add_argument("--stm-vref", type=float, default=STM_VREF_V,
                   help=f"STM32 ADC full-scale V (default {STM_VREF_V})")
    g.add_argument("--ads-vref", type=float, default=ADS_VREF_V,
                   help=f"ADS1220 reference V, i.e. measured AVDD (default {ADS_VREF_V})")
    g.add_argument("--steps-per-rev", type=float, default=200.0,
                   help="stepper steps per revolution in the logged step mode (default 200)")
    g.add_argument("--mm-per-rev", type=float, default=None,
                   help="target travel per motor revolution, to also report mm")
    g.add_argument("--min-dwell", type=float, default=MIN_DWELL_S,
                   help=f"shortest stop at one position used for stationary noise [s] "
                        f"(default {MIN_DWELL_S}; e.g. 0.016 for a sweep with 20 ms steps)")
    g.add_argument("--dwell-settle", type=float, default=DWELL_SETTLE_S,
                   help=f"samples skipped at the start of each dwell while the motor "
                        f"settles [s] (default {DWELL_SETTLE_S})")
    g.add_argument("--true-pos", type=float, nargs="+", default=None,
                   help="known target position(s) in steps, one per feature, for absolute accuracy")
