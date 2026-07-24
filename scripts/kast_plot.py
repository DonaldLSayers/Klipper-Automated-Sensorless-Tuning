#!/usr/bin/env python3
########################################################################
# KAST results plotter
#
# Run this on your workstation (or on the host, if it has matplotlib
# installed) -- NOT inside klippy. klippy only writes the raw CSV;
# rendering happens out-of-process, the same way Klipper's own
# calibrate_shaper.py works.
#
# Usage:
#   python3 scripts/kast_plot.py kast_stepper_x_1234567.csv
#   python3 scripts/kast_plot.py kast_stepper_x_1234567.csv -o out.png
#
# License: GPLv3
########################################################################

import argparse
import csv
import os
import sys


def load_rows(path):
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append({
                'sgt': float(row['sgt']),
                'current': float(row['current']) if row['current'] else None,
                'homing_speed': (float(row['homing_speed'])
                                 if row['homing_speed'] else None),
                'success_rate': float(row['success_rate']),
                'pos_stddev': float(row['pos_stddev']),
                'roughness': (float(row['roughness'])
                              if row['roughness'] else None),
                'score': float(row['score']),
            })
    return rows


def group_by_series(rows):
    """Groups rows by (current, homing_speed) so each combo becomes
    its own line on the SGT axis."""
    series = {}
    for r in rows:
        key = (r['current'], r['homing_speed'])
        series.setdefault(key, []).append(r)
    for key in series:
        series[key].sort(key=lambda r: r['sgt'])
    return series


def series_label(current, homing_speed):
    parts = []
    if current is not None:
        parts.append("%.2fA" % current)
    if homing_speed is not None:
        parts.append("%.0fmm/s" % homing_speed)
    return ", ".join(parts) if parts else "default"


def main():
    parser = argparse.ArgumentParser(
        description="Plot a KAST calibration CSV to a PNG")
    parser.add_argument('csv_path')
    parser.add_argument('-o', '--output', default=None,
                         help="Output PNG path (default: alongside CSV)")
    args = parser.parse_args()

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit(
            "matplotlib is required: pip install matplotlib\n"
            "(run this script on your workstation, not on the printer's "
            "klippy host, unless matplotlib is installed there too)")

    rows = load_rows(args.csv_path)
    if not rows:
        sys.exit("No data rows in %s" % args.csv_path)
    series = group_by_series(rows)
    have_roughness = any(r['roughness'] is not None for r in rows)

    n_plots = 3 if have_roughness else 2
    fig, axes = plt.subplots(n_plots, 1, figsize=(9, 3.2 * n_plots),
                              sharex=True)

    ax_score, ax_success = axes[0], axes[1]
    ax_roughness = axes[2] if have_roughness else None

    for (current, homing_speed), pts in sorted(
            series.items(), key=lambda kv: (kv[0][0] or 0, kv[0][1] or 0)):
        label = series_label(current, homing_speed)
        sgts = [p['sgt'] for p in pts]
        ax_score.plot(sgts, [p['score'] for p in pts], marker='o',
                      label=label)
        ax_success.plot(sgts, [p['success_rate'] * 100 for p in pts],
                         marker='o', label=label)
        if ax_roughness is not None:
            rough = [p['roughness'] for p in pts if p['roughness'] is not None]
            rough_sgts = [p['sgt'] for p in pts if p['roughness'] is not None]
            if rough:
                ax_roughness.plot(rough_sgts, rough, marker='o', label=label)

    ax_score.set_ylabel("Score (lower is better)")
    ax_score.set_title("KAST calibration: %s" % os.path.basename(args.csv_path))
    ax_score.legend(fontsize='small')
    ax_score.grid(True, alpha=0.3)

    ax_success.set_ylabel("Success rate (%)")
    ax_success.set_ylim(-5, 105)
    ax_success.grid(True, alpha=0.3)

    if ax_roughness is not None:
        ax_roughness.set_ylabel("ADXL345 roughness")
        ax_roughness.grid(True, alpha=0.3)

    axes[-1].set_xlabel("driver_SGT / driver_SGTHRS")

    fig.tight_layout()
    out_path = args.output or (os.path.splitext(args.csv_path)[0] + '.png')
    fig.savefig(out_path, dpi=150)
    print("Wrote %s" % out_path)


if __name__ == '__main__':
    main()
