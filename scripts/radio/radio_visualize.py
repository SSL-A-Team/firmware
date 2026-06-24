#!/usr/bin/env python3
"""
Radio latency visualizer for A-Team radio profiler JSON output.

Single run:
  python3 radio_visualize.py run.json

Compare two runs (Odin vs Nora, different rates, etc.):
  python3 radio_visualize.py odin.json nora.json --labels Odin Nora

Save to file instead of interactive window:
  python3 radio_visualize.py run.json --output latency.png
  python3 radio_visualize.py odin.json nora.json --labels Odin Nora --output cmp.pdf
"""

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional

try:
    import matplotlib
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except ImportError:
    print("matplotlib required:  pip install matplotlib", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Palette — distinct even on colourblind displays
# ---------------------------------------------------------------------------
COLORS = ["#2c7bb6", "#d7191c", "#1a9641", "#f1a340", "#7b2d8b", "#878787"]


# ---------------------------------------------------------------------------
# Data helpers
# ---------------------------------------------------------------------------

def load(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def cdf(samples: List[float]):
    s = sorted(samples)
    n = len(s)
    return s, [(i + 1) / n for i in range(n)]


def percentile(samples: List[float], p: float) -> float:
    if not samples:
        return float("nan")
    s = sorted(samples)
    idx = (len(s) - 1) * p / 100.0
    lo = int(idx)
    hi = lo + 1
    frac = idx - lo
    if hi >= len(s):
        return s[lo]
    return s[lo] * (1 - frac) + s[hi] * frac


def downsample(data: List[float], max_pts: int = 5000) -> List[float]:
    """Uniform stride downsample for time-series plots."""
    if len(data) <= max_pts:
        return data
    step = len(data) / max_pts
    return [data[int(i * step)] for i in range(max_pts)]


def clip_to(samples: List[float], pct: float = 99.5) -> List[float]:
    """Return samples with extreme tail trimmed at given percentile."""
    if not samples:
        return samples
    limit = percentile(samples, pct)
    return [x for x in samples if x <= limit]


def fmt_f(v, digits=3) -> str:
    if v is None or (isinstance(v, float) and v != v):  # nan check
        return "—"
    return f"{v:.{digits}f}"


# ---------------------------------------------------------------------------
# Single-run dashboard  (3 × 3 grid)
# ---------------------------------------------------------------------------

def plot_single(data: dict, title: Optional[str], out: Optional[str]):
    rtt = data.get("rtt_ms", {})
    jitter = data.get("jitter_ms", {})
    packets = data.get("packets", {})
    meta = data.get("meta", {})

    rtt_s = rtt.get("samples", [])
    jit_s = jitter.get("samples", [])

    rate = meta.get("rate_hz", "?")
    ts = meta.get("timestamp", "")[:19]
    ip = meta.get("robot_ip", "?")
    payload = meta.get("payload_size_bytes", "?")
    fig_title = title or f"Radio Latency  |  {rate} Hz  ·  {payload} B  ·  {ip}  ·  {ts}"

    fig = plt.figure(figsize=(17, 10))
    fig.suptitle(fig_title, fontsize=12, fontweight="bold", y=0.99)
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.50, wspace=0.38)

    # ── RTT time series (top row, 2 cols) ──────────────────────────────────
    ax_ts = fig.add_subplot(gs[0, :2])
    if rtt_s:
        ds = downsample(rtt_s)
        cap = percentile(rtt_s, 99.5)
        ds_clipped = [min(x, cap) for x in ds]
        n_clipped = sum(1 for x in rtt_s if x > cap)
        ax_ts.plot(ds_clipped, linewidth=0.5, color=COLORS[0], alpha=0.75)
        p99v = rtt.get("p99")
        if p99v:
            ax_ts.axhline(p99v, color="red", linewidth=0.8, linestyle="--",
                          label=f"p99 = {p99v:.1f} ms")
        ax_ts.axhline(rtt.get("mean", 0), color="orange", linewidth=0.8, linestyle="-.",
                      label=f"mean = {rtt.get('mean', 0):.1f} ms")
        ax_ts.legend(fontsize=7, loc="upper right")
        suffix = f"  (y capped at p99.5 = {cap:.1f} ms; {n_clipped} pts above)" if n_clipped else ""
        ax_ts.set_title(f"RTT over time{suffix}", fontsize=9)
    ax_ts.set_xlabel("Packet index (downsampled)", fontsize=8)
    ax_ts.set_ylabel("RTT (ms)", fontsize=8)
    ax_ts.tick_params(labelsize=7)

    # ── RTT histogram (top-right) ───────────────────────────────────────────
    ax_hist = fig.add_subplot(gs[0, 2])
    if rtt_s:
        hclip = clip_to(rtt_s, 99.5)
        ax_hist.hist(hclip, bins=60, color=COLORS[0], edgecolor="none", alpha=0.85)
        ax_hist.axvline(rtt.get("mean", 0), color="orange", linewidth=1.2,
                        label=f"mean {rtt.get('mean',0):.1f}")
        ax_hist.axvline(rtt.get("p50", 0), color="green", linewidth=1.2,
                        label=f"p50  {rtt.get('p50',0):.1f}")
        ax_hist.legend(fontsize=7)
    ax_hist.set_xlabel("RTT (ms)", fontsize=8)
    ax_hist.set_ylabel("Count", fontsize=8)
    ax_hist.set_title("RTT histogram (≤ p99.5)", fontsize=9)
    ax_hist.tick_params(labelsize=7)

    # ── RTT CDF (middle-left) ──────────────────────────────────────────────
    ax_cdf = fig.add_subplot(gs[1, 0])
    if rtt_s:
        xs, ys = cdf(rtt_s)
        ax_cdf.plot(xs, ys, linewidth=1.2, color=COLORS[0])
        for pname, col in [("p50", "green"), ("p95", "orange"), ("p99", "red")]:
            pv = rtt.get(pname)
            if pv:
                ax_cdf.axvline(pv, linewidth=0.8, linestyle=":", color=col,
                               label=f"{pname} = {pv:.1f}")
        ax_cdf.set_xlim(left=0, right=percentile(rtt_s, 99.5) * 1.3)
        ax_cdf.legend(fontsize=7)
    ax_cdf.set_xlabel("RTT (ms)", fontsize=8)
    ax_cdf.set_ylabel("CDF", fontsize=8)
    ax_cdf.set_title("RTT CDF", fontsize=9)
    ax_cdf.tick_params(labelsize=7)

    # ── Jitter time series (middle, 2 cols) ────────────────────────────────
    ax_jit = fig.add_subplot(gs[1, 1:])
    if jit_s:
        dsj = downsample(jit_s)
        jcap = percentile(jit_s, 99.5)
        dsj_clipped = [min(x, jcap) for x in dsj]
        ax_jit.plot(dsj_clipped, linewidth=0.5, color=COLORS[1], alpha=0.75)
        jmean = jitter.get("mean")
        if jmean:
            ax_jit.axhline(jmean, color="orange", linewidth=0.8, linestyle="-.",
                           label=f"mean = {jmean:.2f} ms")
            ax_jit.legend(fontsize=7, loc="upper right")
    ax_jit.set_xlabel("Packet index (downsampled)", fontsize=8)
    ax_jit.set_ylabel("Jitter (ms)", fontsize=8)
    ax_jit.set_title("Jitter over time  |RTT[i] − RTT[i−1]|  (y capped at p99.5)", fontsize=9)
    ax_jit.tick_params(labelsize=7)

    # ── Packet counts bar (bottom-left) ────────────────────────────────────
    ax_bar = fig.add_subplot(gs[2, 0])
    bar_labels = ["Echoed", "Dropped", "OOO"]
    bar_vals = [
        packets.get("echoed", 0),
        packets.get("dropped", 0),
        packets.get("out_of_order", 0),
    ]
    bar_colors = ["#4dac26", "#d01c8b", "#f1a340"]
    bars = ax_bar.bar(bar_labels, bar_vals, color=bar_colors, edgecolor="none", width=0.5)
    for bar, v in zip(bars, bar_vals):
        ax_bar.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + max(bar_vals) * 0.01,
                    f"{v:,}", ha="center", va="bottom", fontsize=7)
    ax_bar.set_title(f"Packet counts  (sent = {packets.get('sent', 0):,})", fontsize=9)
    ax_bar.tick_params(labelsize=7)

    # ── Summary table (bottom, 2 cols) ─────────────────────────────────────
    ax_tbl = fig.add_subplot(gs[2, 1:])
    ax_tbl.axis("off")
    rows = [
        ["Rate",          f"{rate} Hz"],
        ["Payload",       f"{payload} B"],
        ["Duration",      f"{meta.get('duration_s', '?')} s"],
        ["Sent",          f"{packets.get('sent', 0):,}"],
        ["Echoed",        f"{packets.get('echoed', 0):,}  ({packets.get('echoed_pct', 0):.2f}%)"],
        ["Dropped",       f"{packets.get('dropped', 0):,}  ({packets.get('dropped_pct', 0):.2f}%)"],
        ["Max burst loss",str(packets.get("max_burst_loss", 0))],
        ["RTT mean",      f"{fmt_f(rtt.get('mean'))} ms"],
        ["RTT stddev",    f"{fmt_f(rtt.get('stddev'))} ms"],
        ["RTT p50",       f"{fmt_f(rtt.get('p50'))} ms"],
        ["RTT p95",       f"{fmt_f(rtt.get('p95'))} ms"],
        ["RTT p99",       f"{fmt_f(rtt.get('p99'))} ms"],
        ["RTT min / max", f"{fmt_f(rtt.get('min'))} / {fmt_f(rtt.get('max'))} ms"],
        ["Jitter mean",   f"{fmt_f(jitter.get('mean'))} ms"],
        ["Jitter max",    f"{fmt_f(jitter.get('max'))} ms"],
    ]
    tbl = ax_tbl.table(
        cellText=rows,
        colLabels=["Metric", "Value"],
        cellLoc="left",
        loc="center",
        bbox=[0, 0, 1, 1],
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#cccccc")
        if r == 0:
            cell.set_facecolor("#303030")
            cell.set_text_props(color="white", fontweight="bold")
        elif r % 2 == 0:
            cell.set_facecolor("#f2f2f2")
    ax_tbl.set_title("Summary", fontsize=9, pad=4)

    _save_or_show(fig, out)


# ---------------------------------------------------------------------------
# Multi-run comparison  (2 × 3 grid)
# ---------------------------------------------------------------------------

def plot_compare(runs: List[dict], labels: List[str], title: Optional[str], out: Optional[str]):
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(title or "Radio Latency Comparison", fontsize=13, fontweight="bold", y=0.99)
    plt.subplots_adjust(hspace=0.42, wspace=0.35)

    ax_cdf, ax_hist, ax_box = axes[0]
    ax_jit_hist, ax_pkt, ax_tbl = axes[1]

    max_p99 = 0.0

    for i, (data, label) in enumerate(zip(runs, labels)):
        color = COLORS[i % len(COLORS)]
        rtt = data.get("rtt_ms", {})
        jitter = data.get("jitter_ms", {})
        rtt_s = rtt.get("samples", [])
        jit_s = jitter.get("samples", [])

        if rtt_s:
            # CDF
            xs, ys = cdf(rtt_s)
            ax_cdf.plot(xs, ys, color=color, linewidth=1.3, label=label)
            p99v = rtt.get("p99", 0) or 0
            max_p99 = max(max_p99, p99v)
            ax_cdf.axvline(p99v, color=color, linewidth=0.7, linestyle=":", alpha=0.7)

            # Histogram
            hclip = clip_to(rtt_s, 99.5)
            ax_hist.hist(hclip, bins=60, color=color, alpha=0.5, label=label, edgecolor="none")

        if jit_s:
            jclip = clip_to(jit_s, 99.5)
            ax_jit_hist.hist(jclip, bins=60, color=color, alpha=0.5, label=label, edgecolor="none")

    # CDF axis
    ax_cdf.set_xlim(left=0, right=max_p99 * 1.5 if max_p99 else None)
    ax_cdf.legend(fontsize=8)
    ax_cdf.set_xlabel("RTT (ms)", fontsize=8)
    ax_cdf.set_ylabel("CDF", fontsize=8)
    ax_cdf.set_title("RTT CDF  (dotted = p99 per run)", fontsize=9)
    ax_cdf.tick_params(labelsize=7)

    # Histogram axis
    ax_hist.legend(fontsize=8)
    ax_hist.set_xlabel("RTT (ms)", fontsize=8)
    ax_hist.set_ylabel("Count", fontsize=8)
    ax_hist.set_title("RTT histogram (≤ p99.5 each)", fontsize=9)
    ax_hist.tick_params(labelsize=7)

    # Box plot (cap each series at its own p99.5 to keep y scale readable)
    box_data = []
    for data in runs:
        s = data.get("rtt_ms", {}).get("samples", [])
        box_data.append(clip_to(s, 99.5))
    import matplotlib
    _mpl_ver = tuple(int(x) for x in matplotlib.__version__.split(".")[:2])
    _box_label_kwarg = "tick_labels" if _mpl_ver >= (3, 9) else "labels"
    bp = ax_box.boxplot(
        box_data, **{_box_label_kwarg: labels}, patch_artist=True, showfliers=False,
        medianprops=dict(color="black", linewidth=1.5),
    )
    for patch, color in zip(bp["boxes"], COLORS):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    ax_box.set_ylabel("RTT (ms)", fontsize=8)
    ax_box.set_title("RTT box plot (IQR, no outliers, capped at p99.5)", fontsize=9)
    ax_box.tick_params(labelsize=7, axis="x", rotation=15)

    # Jitter histogram
    ax_jit_hist.legend(fontsize=8)
    ax_jit_hist.set_xlabel("Jitter (ms)", fontsize=8)
    ax_jit_hist.set_ylabel("Count", fontsize=8)
    ax_jit_hist.set_title("Jitter histogram (≤ p99.5 each)", fontsize=9)
    ax_jit_hist.tick_params(labelsize=7)

    # Grouped bar: echoed% and dropped%
    bar_groups = ["Echoed %", "Dropped %"]
    bar_keys = ["echoed_pct", "dropped_pct"]
    x = list(range(len(bar_groups)))
    w = 0.8 / max(len(runs), 1)
    for i, (data, label) in enumerate(zip(runs, labels)):
        color = COLORS[i % len(COLORS)]
        pkts = data.get("packets", {})
        vals = [pkts.get(k, 0) for k in bar_keys]
        offsets = [xi + i * w - (len(runs) - 1) * w / 2 for xi in x]
        ax_pkt.bar(offsets, vals, width=w * 0.88, color=color, alpha=0.82, label=label, edgecolor="none")
    ax_pkt.set_xticks(x)
    ax_pkt.set_xticklabels(bar_groups, fontsize=8)
    ax_pkt.set_ylabel("%", fontsize=8)
    ax_pkt.set_title("Packet rates (%)", fontsize=9)
    ax_pkt.legend(fontsize=7)
    ax_pkt.tick_params(labelsize=7)

    # Summary comparison table
    ax_tbl.axis("off")
    col_labels = ["Metric"] + labels
    rows = [
        ["Rate (Hz)"]      + [str(d.get("meta", {}).get("rate_hz", "?")) for d in runs],
        ["Payload (B)"]    + [str(d.get("meta", {}).get("payload_size_bytes", "?")) for d in runs],
        ["Sent"]           + [f"{d.get('packets',{}).get('sent',0):,}" for d in runs],
        ["Echoed %"]       + [f"{d.get('packets',{}).get('echoed_pct',0):.2f}%" for d in runs],
        ["Dropped %"]      + [f"{d.get('packets',{}).get('dropped_pct',0):.2f}%" for d in runs],
        ["Max burst loss"] + [str(d.get("packets",{}).get("max_burst_loss",0)) for d in runs],
        ["RTT mean (ms)"]  + [fmt_f(d.get("rtt_ms",{}).get("mean")) for d in runs],
        ["RTT stddev (ms)"]+ [fmt_f(d.get("rtt_ms",{}).get("stddev")) for d in runs],
        ["RTT p50 (ms)"]   + [fmt_f(d.get("rtt_ms",{}).get("p50")) for d in runs],
        ["RTT p95 (ms)"]   + [fmt_f(d.get("rtt_ms",{}).get("p95")) for d in runs],
        ["RTT p99 (ms)"]   + [fmt_f(d.get("rtt_ms",{}).get("p99")) for d in runs],
        ["RTT min (ms)"]   + [fmt_f(d.get("rtt_ms",{}).get("min")) for d in runs],
        ["RTT max (ms)"]   + [fmt_f(d.get("rtt_ms",{}).get("max")) for d in runs],
        ["Jitter mean (ms)"]+ [fmt_f(d.get("jitter_ms",{}).get("mean")) for d in runs],
        ["Jitter max (ms)"] + [fmt_f(d.get("jitter_ms",{}).get("max")) for d in runs],
    ]
    tbl = ax_tbl.table(
        cellText=rows,
        colLabels=col_labels,
        cellLoc="center",
        loc="center",
        bbox=[0, 0, 1, 1],
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#cccccc")
        if r == 0:
            cell.set_facecolor("#303030")
            cell.set_text_props(color="white", fontweight="bold")
        elif r % 2 == 0:
            cell.set_facecolor("#f2f2f2")
    ax_tbl.set_title("Summary", fontsize=9, pad=4)

    _save_or_show(fig, out)


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def _ensure_interactive_backend():
    """Switch away from Agg to a windowed backend. Tries common backends in order."""
    if matplotlib.get_backend().lower() != "agg":
        return
    for backend in ("TkAgg", "Qt5Agg", "QtAgg", "GTK4Agg", "GTK3Agg", "wxAgg", "MacOSX"):
        try:
            plt.switch_backend(backend)
            if matplotlib.get_backend().lower() != "agg":
                return
        except Exception:
            continue
    print(
        "No interactive display backend found (tried Tk, Qt, GTK, wx).\n"
        "Save to file instead:  --output latency.png",
        file=sys.stderr,
    )
    sys.exit(1)


def _save_or_show(fig, out: Optional[str]):
    matplotlib.rcParams["axes.spines.top"] = False
    matplotlib.rcParams["axes.spines.right"] = False
    if out:
        fig.savefig(out, dpi=150, bbox_inches="tight")
        print(f"Saved → {out}")
    else:
        _ensure_interactive_backend()
        plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Visualize A-Team radio latency profiler output",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "files", nargs="+", metavar="JSON",
        help="One or more JSON result files from radio_latency.py --json",
    )
    parser.add_argument(
        "--labels", nargs="+", metavar="LABEL",
        help="Display label for each file (default: filename stem)",
    )
    parser.add_argument(
        "--title", type=str, default=None,
        help="Override figure title",
    )
    parser.add_argument(
        "--output", "-o", type=str, default=None, metavar="PATH",
        help="Save to file (PNG / PDF / SVG) instead of interactive window",
    )
    args = parser.parse_args()

    runs = []
    for path in args.files:
        try:
            runs.append(load(path))
        except FileNotFoundError:
            print(f"File not found: {path}", file=sys.stderr)
            sys.exit(1)
        except json.JSONDecodeError as e:
            print(f"Bad JSON in {path}: {e}", file=sys.stderr)
            sys.exit(1)

    labels = list(args.labels or [])
    for i in range(len(labels), len(runs)):
        labels.append(Path(args.files[i]).stem)

    if len(runs) == 1:
        plot_single(runs[0], args.title, args.output)
    else:
        plot_compare(runs, labels, args.title, args.output)


if __name__ == "__main__":
    main()
