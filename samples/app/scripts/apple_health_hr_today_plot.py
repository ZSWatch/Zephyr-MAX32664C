#!/usr/bin/env python3
"""
Extract Apple Watch heart rate samples for the current day from an Apple Health
`export.xml` file and plot beats per minute over time.
"""

import argparse
import datetime as dt
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, List, Tuple

import matplotlib.pyplot as plt
from matplotlib.dates import DateFormatter

# Apple encodes timestamps like "2023-10-12 14:05:12 +0200".
APPLE_HEALTH_TIMESTAMP = "%Y-%m-%d %H:%M:%S %z"


def iter_heart_rate_samples(xml_path: Path) -> Iterable[Tuple[dt.datetime, float, str]]:
    """Yield (timestamp, bpm, source_name) tuples for heart rate records."""
    # iterparse streams the file without loading the full XML tree into memory.
    context = ET.iterparse(xml_path, events=("start", "end"))
    _, root = next(context)  # Grab root to manually clear processed elements.

    for event, elem in context:
        if event == "end" and elem.tag == "Record":
            if elem.get("type") == "HKQuantityTypeIdentifierHeartRate":
                try:
                    timestamp = dt.datetime.strptime(elem.get("startDate"), APPLE_HEALTH_TIMESTAMP)
                    bpm = float(elem.get("value"))
                    source = elem.get("sourceName", "Unknown")
                    yield timestamp, bpm, source
                except (TypeError, ValueError):
                    # Skip malformed records but continue parsing.
                    pass
            elem.clear()
            root.clear()


def filter_samples_for_day(
    samples: Iterable[Tuple[dt.datetime, float, str]], target_date: dt.date
) -> List[Tuple[dt.datetime, float, str]]:
    """Return samples where the timestamp falls on the target_date (local time)."""
    results: List[Tuple[dt.datetime, float, str]] = []
    for timestamp, bpm, source in samples:
        local_ts = timestamp.astimezone()  # Convert to system local time zone.
        if local_ts.date() == target_date:
            results.append((local_ts, bpm, source))
    return results


def plot_day(samples: List[Tuple[dt.datetime, float, str]], target_date: dt.date) -> None:
    """Plot heart rate over time for the selected day."""
    print("Plotting heart rate samples...", len(samples))
    times = [sample[0] for sample in samples]
    bpm_values = [sample[1] for sample in samples]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, bpm_values, marker="o", linestyle="-", linewidth=1, markersize=3)
    ax.set_title(f"Apple Watch Heart Rate on {target_date.isoformat()}")
    ax.set_xlabel("Time")
    ax.set_ylabel("Heart Rate (bpm)")

    ax.xaxis.set_major_formatter(DateFormatter("%H:%M"))
    fig.autofmt_xdate()
    ax.grid(True, which="major", linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot Apple Watch heart rate samples from an Apple Health export for today."
    )
    parser.add_argument("export_xml", type=Path, help="Path to Apple Health export.xml file")
    parser.add_argument(
        "--date",
        type=str,
        help="Optional date (YYYY-MM-DD). Defaults to today in the local time zone.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    export_path: Path = args.export_xml

    if not export_path.exists():
        print(f"error: {export_path} does not exist", file=sys.stderr)
        return 1

    try:
        target_date = (
            dt.date.fromisoformat(args.date)
            if args.date
            else dt.datetime.now().astimezone().date()
        )
    except ValueError:
        print("error: --date must be formatted as YYYY-MM-DD", file=sys.stderr)
        return 1

    samples = filter_samples_for_day(iter_heart_rate_samples(export_path), target_date)
    if not samples:
        print(f"No heart rate samples found for {target_date.isoformat()}.")
        return 0

    samples.sort(key=lambda sample: sample[0])
    print(f"Found {len(samples)} samples for {target_date.isoformat()} from {export_path}.")
    plot_day(samples, target_date)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
