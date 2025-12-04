#!/usr/bin/env python3
"""Compare hardware evaluation snapshot CSVs."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

OPTICAL_CHANNELS = ["GREEN1", "GREEN2"]
METRICS = [
    "snr_db",
    "peak_width_hz",
    "peak_power_db",
    "noise_floor_db",
    "ac_dc_ratio",
    "clipping_pct",
]

IMPROVEMENT_THRESHOLDS = {
    "snr_db": 3.0,  # dB
    "peak_width_hz": -0.1,  # Hz (negative is narrower)
    "clipping_pct": -0.5,  # percentage points (negative is less clipping)
}

REGRESSION_THRESHOLDS = {
    "snr_db": -3.0,
    "peak_width_hz": 0.1,
    "clipping_pct": 0.5,
}


def parse_float(value: str) -> Optional[float]:
    if value is None:
        return None
    value = value.strip()
    if not value:
        return None
    try:
        numeric = float(value)
    except ValueError:
        return None
    if math.isfinite(numeric):
        return numeric
    return None


def derive_motion_bin(percentile: Optional[float]) -> Optional[str]:
    if percentile is None:
        return None
    percentile = max(0.0, min(100.0, percentile))
    if percentile < 34.0:
        return "low"
    if percentile < 67.0:
        return "mid"
    return "high"


def load_snapshot_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8") as handle:
        reader = csv.DictReader(line for line in handle if not line.lstrip().startswith("#"))
        return list(reader)


def aggregate_metrics(
    rows: Iterable[Dict[str, str]],
    use_motion_bins: bool,
) -> Tuple[
    Dict[str, Dict[Tuple[str, ...], Dict[str, np.ndarray]]],
    Dict[Tuple[str, ...], np.ndarray],
]:
    accumulator: Dict[str, Dict[Tuple[str, ...], Dict[str, List[float]]]] = {
        channel: defaultdict(lambda: defaultdict(list)) for channel in OPTICAL_CHANNELS
    }
    motion_stats: Dict[Tuple[str, ...], List[float]] = defaultdict(list)

    for row in rows:
        segment = row.get("segment", "").strip() or "unknown"
        percentile = parse_float(row.get("a_mag_percentile"))
        if use_motion_bins:
            motion = derive_motion_bin(percentile)
            if motion is None:
                continue
            group_key: Tuple[str, ...] = (segment, motion)
        else:
            group_key = (segment,)

        for channel in OPTICAL_CHANNELS:
            prefix = channel.lower()
            channel_bucket = accumulator[channel][group_key]

            snr = parse_float(row.get(f"{prefix}_snr_db"))
            peak_power = parse_float(row.get(f"{prefix}_peak_power_db"))
            peak_width = parse_float(row.get(f"{prefix}_peak_width_hz"))
            ac_rms = parse_float(row.get(f"{prefix}_ac_rms"))
            dc_level = parse_float(row.get(f"{prefix}_dc_level"))
            ac_dc_ratio = parse_float(row.get(f"{prefix}_ac_dc_ratio"))
            clipping_pct = parse_float(row.get(f"{prefix}_clipping_pct"))

            if snr is not None:
                channel_bucket["snr_db"].append(snr)
            if peak_width is not None:
                channel_bucket["peak_width_hz"].append(peak_width)
            if peak_power is not None:
                channel_bucket["peak_power_db"].append(peak_power)
            if snr is not None and peak_power is not None:
                noise_floor = peak_power - snr
                channel_bucket["noise_floor_db"].append(noise_floor)
            if ac_dc_ratio is not None:
                channel_bucket["ac_dc_ratio"].append(ac_dc_ratio)
            if clipping_pct is not None:
                channel_bucket["clipping_pct"].append(clipping_pct)

        a_mag_mean = parse_float(row.get("a_mag_mean"))
        if a_mag_mean is not None:
            motion_stats[group_key].append(a_mag_mean)

    aggregated: Dict[str, Dict[Tuple[str, ...], Dict[str, np.ndarray]]] = {
        channel: {} for channel in OPTICAL_CHANNELS
    }
    for channel, groups in accumulator.items():
        for group_key, metrics in groups.items():
            aggregated[channel][group_key] = {
                metric: np.asarray(values, dtype=float)
                for metric, values in metrics.items()
                if values
            }
    motion_arrays = {
        key: np.asarray(values, dtype=float) for key, values in motion_stats.items() if values
    }
    return aggregated, motion_arrays


def percentile_distance(values: np.ndarray, p_high: float, p_low: float) -> float:
    return float(np.percentile(values, p_high) - np.percentile(values, p_low))


def bootstrap_delta(
    values_a: np.ndarray,
    values_b: np.ndarray,
    rng: np.random.Generator,
    n_boot: int = 1000,
) -> Tuple[float, float]:
    if values_a.size == 0 or values_b.size == 0:
        return math.nan, math.nan

    idx_a = rng.integers(0, values_a.size, size=(n_boot, values_a.size))
    idx_b = rng.integers(0, values_b.size, size=(n_boot, values_b.size))
    medians_a = np.median(values_a[idx_a], axis=1)
    medians_b = np.median(values_b[idx_b], axis=1)
    deltas = medians_b - medians_a
    return float(np.percentile(deltas, 2.5)), float(np.percentile(deltas, 97.5))


def evaluate_flag(metric: str, delta: float) -> str:
    improvement = IMPROVEMENT_THRESHOLDS.get(metric)
    regression = REGRESSION_THRESHOLDS.get(metric)

    if improvement is not None:
        if (metric in ("snr_db",) and delta >= improvement) or (
            metric in ("peak_width_hz", "clipping_pct") and delta <= improvement
        ):
            return "improved"

    if regression is not None:
        if (metric in ("snr_db",) and delta <= regression) or (
            metric in ("peak_width_hz", "clipping_pct") and delta >= regression
        ):
            return "regressed"

    return ""


def build_results(
    aggregated_a: Dict[str, Dict[Tuple[str, ...], Dict[str, np.ndarray]]],
    aggregated_b: Dict[str, Dict[Tuple[str, ...], Dict[str, np.ndarray]]],
    motion_a: Dict[Tuple[str, ...], np.ndarray],
    motion_b: Dict[Tuple[str, ...], np.ndarray],
    use_motion_bins: bool,
    motion_warnings: List[str],
) -> List[Dict[str, object]]:
    results: List[Dict[str, object]] = []
    rng = np.random.default_rng(42)

    for channel in OPTICAL_CHANNELS:
        groups_a = set(aggregated_a.get(channel, {}).keys())
        groups_b = set(aggregated_b.get(channel, {}).keys())
        for group_key in sorted(groups_a | groups_b):
            metrics_a = aggregated_a.get(channel, {}).get(group_key, {})
            metrics_b = aggregated_b.get(channel, {}).get(group_key, {})

            for metric in METRICS:
                values_a = metrics_a.get(metric)
                values_b = metrics_b.get(metric)
                if values_a is None or values_b is None or values_a.size == 0 or values_b.size == 0:
                    continue

                motion_label = group_key[1] if len(group_key) > 1 else "all"
                median_a = float(np.median(values_a))
                median_b = float(np.median(values_b))
                iqr_a = percentile_distance(values_a, 75.0, 25.0)
                iqr_b = percentile_distance(values_b, 75.0, 25.0)
                delta = median_b - median_a
                ci_low, ci_high = bootstrap_delta(values_a, values_b, rng)
                flag = evaluate_flag(metric, delta)

                results.append(
                    {
                        "segment": group_key[0],
                        "motion_bin": motion_label,
                        "channel": channel,
                        "metric": metric,
                        "median_a": median_a,
                        "iqr_a": iqr_a,
                        "median_b": median_b,
                        "iqr_b": iqr_b,
                        "delta": delta,
                        "ci_low": ci_low,
                        "ci_high": ci_high,
                        "n_a": int(values_a.size),
                        "n_b": int(values_b.size),
                        "flag": flag,
                    }
                )

    if not use_motion_bins:
        for group_key in sorted(set(motion_a.keys()) | set(motion_b.keys())):
            values_a = motion_a.get(group_key)
            values_b = motion_b.get(group_key)
            if values_a is None or values_b is None or values_a.size == 0 or values_b.size == 0:
                continue
            motion_label = group_key[1] if len(group_key) > 1 else "all"
            median_a = float(np.median(values_a))
            median_b = float(np.median(values_b))
            iqr_a = percentile_distance(values_a, 75.0, 25.0)
            iqr_b = percentile_distance(values_b, 75.0, 25.0)
            delta = median_b - median_a
            ci_low, ci_high = bootstrap_delta(values_a, values_b, rng)
            rel_diff = abs(delta) / median_a if median_a else float("inf")
            if rel_diff > 0.15:
                motion_warnings.append(
                    f"|A| mean differs by {rel_diff*100:.1f}% in segment '{group_key[0]}' "
                    f"(A={median_a:.1f}, B={median_b:.1f})"
                )
            results.append(
                {
                    "segment": group_key[0],
                    "motion_bin": motion_label,
                    "channel": "A_MAG",
                    "metric": "a_mag_mean",
                    "median_a": median_a,
                    "iqr_a": iqr_a,
                    "median_b": median_b,
                    "iqr_b": iqr_b,
                    "delta": delta,
                    "ci_low": ci_low,
                    "ci_high": ci_high,
                    "n_a": int(values_a.size),
                    "n_b": int(values_b.size),
                    "flag": "",
                }
            )

    results.sort(key=lambda r: (r["segment"], r["motion_bin"], r["channel"], r["metric"]))
    return results


def format_float(value: Optional[float], precision: int = 4) -> str:
    if value is None or not math.isfinite(value):
        return ""
    return f"{value:.{precision}f}"


def write_csv(path: Path, results: List[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "segment",
        "motion_bin",
        "channel",
        "metric",
        "median_a",
        "iqr_a",
        "median_b",
        "iqr_b",
        "delta",
        "ci_low",
        "ci_high",
        "n_a",
        "n_b",
        "flag",
    ]
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in results:
            out_row = row.copy()
            for key in ("median_a", "iqr_a", "median_b", "iqr_b", "delta", "ci_low", "ci_high"):
                out_row[key] = format_float(out_row[key])
            writer.writerow(out_row)


def write_markdown(
    path: Path,
    results: List[Dict[str, object]],
    file_a: Path,
    file_b: Path,
    warnings: List[str],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines: List[str] = []
    lines.append("# Snapshot Comparison Report")
    lines.append("")
    lines.append(f"- Dataset A: `{file_a}`")
    lines.append(f"- Dataset B: `{file_b}`")
    lines.append("")

    if warnings:
        lines.append("## Warnings")
        lines.append("")
        for item in warnings:
            lines.append(f"- {item}")
        lines.append("")

    flagged = [row for row in results if row["flag"]]
    if flagged:
        lines.append("## Flagged Metrics")
        lines.append("")
        lines.append("| Segment | Motion | Channel | Metric | Delta (B-A) | 95% CI | Samples (A/B) | Flag |")
        lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
        for row in flagged:
            delta_str = format_float(row["delta"], 3) or "--"
            ci_low = format_float(row["ci_low"], 3) or "--"
            ci_high = format_float(row["ci_high"], 3) or "--"
            lines.append(
                "| {segment} | {motion} | {channel} | {metric} | {delta} | [{low}, {high}] | {n_a}/{n_b} | {flag} |".format(
                    segment=row["segment"],
                    motion=row["motion_bin"],
                    channel=row["channel"],
                    metric=row["metric"],
                    delta=delta_str,
                    low=ci_low,
                    high=ci_high,
                    n_a=row["n_a"],
                    n_b=row["n_b"],
                    flag=row["flag"],
                )
            )
    else:
        lines.append("No metrics met the improvement/regression thresholds.")

    lines.append("")
    lines.append("Report generated by `compare_snapshots.py`. Use the CSV output for deeper analysis.")
    lines.append("")

    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare hardware-eval snapshot CSV files.")
    parser.add_argument("snapshot_a", type=Path, help="Baseline snapshot CSV (dataset A)")
    parser.add_argument("snapshot_b", type=Path, help="Comparison snapshot CSV (dataset B)")
    parser.add_argument(
        "--out-csv",
        type=Path,
        default=Path("comparison_summary.csv"),
        help="Output CSV file for machine-readable summary",
    )
    parser.add_argument(
        "--out-report",
        type=Path,
        default=Path("comparison_report.md"),
        help="Output Markdown report file",
    )
    parser.add_argument(
        "--grouping",
        choices=["bins", "segments"],
        default="bins",
        help="Comparison grouping strategy: 'bins' (default) uses motion bins, 'segments' ignores bins.",
    )
    args = parser.parse_args()

    rows_a = load_snapshot_rows(args.snapshot_a)
    rows_b = load_snapshot_rows(args.snapshot_b)
    use_bins = args.grouping == "bins"
    aggregated_a, motion_a = aggregate_metrics(rows_a, use_bins)
    aggregated_b, motion_b = aggregate_metrics(rows_b, use_bins)

    motion_warnings: List[str] = []
    results = build_results(aggregated_a, aggregated_b, motion_a, motion_b, use_bins, motion_warnings)
    write_csv(args.out_csv, results)
    def median_from_rows(field: str, rows: List[Dict[str, str]]) -> Optional[float]:
        values = [parse_float(row.get(field, "")) for row in rows]
        filtered = [val for val in values if val is not None]
        if not filtered:
            return None
        return float(np.median(filtered))

    warnings: List[str] = []
    for field in ("fs_hz", "nperseg", "delta_f_hz"):
        median_a = median_from_rows(field, rows_a)
        median_b = median_from_rows(field, rows_b)
        if median_a is None or median_b is None:
            continue
        if median_a == 0:
            continue
        rel_diff = abs(median_a - median_b) / abs(median_a)
        if rel_diff > 0.10:
            warnings.append(
                f"Median {field} differs by {rel_diff*100:.1f}% (A={median_a:.3f}, B={median_b:.3f})"
            )

    warnings.extend(motion_warnings)
    write_markdown(args.out_report, results, args.snapshot_a, args.snapshot_b, warnings)


if __name__ == "__main__":
    main()
