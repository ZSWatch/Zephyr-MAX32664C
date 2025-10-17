#!/usr/bin/env python3
"""
Heart Rate Log Plotter
Parses and plots heart rate sensor data from log files.
Usage: python hr_plotter.py <log_file_path>
"""

import re
import sys
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.ticker import MaxNLocator
import numpy as np


def parse_log_file(file_path):
    """
    Parse the log file and extract HR, Confidence, Skin Contact, and Activity Class data.

    Example: HR,70,bpm;HR_Conf,0,;RR,0,ms;RR_Conf,0,;SC,1,;Activity,0,;SpO2,0,%;SpO2_Conf,0,

    Returns:
        dict: Contains lists of timestamps, hr, confidence, skin_contact, and activity_class values
    """
    data = {
        "timestamps": [],
        "hr": [],
        "confidence": [],
        "skin_contact": [],
        "activity_class": [],
        "button_press_times": [],
    }

    # Pattern to match the new CSV-like format with ISO timestamp
    # Format: 2025-10-17T10:42:03.384Z HR,70,bpm;HR_Conf,0,;RR,0,ms;...
    data_pattern = r"^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z)\s+(.+)$"

    # Pattern to match button press lines
    # Format: 2025-10-17T10:42:15.049Z [00:00:24.588,019] main: Button pressed!
    button_pattern = (
        r"^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z)\s+\[.+\]\s+.*Button pressed!"
    )

    with open(file_path, "r") as f:
        for line in f:
            line = line.strip()

            # Check for button press
            button_match = re.match(button_pattern, line)
            if button_match:
                timestamp_str = button_match.group(1)
                time_obj = datetime.strptime(timestamp_str, "%Y-%m-%dT%H:%M:%S.%fZ")
                data["button_press_times"].append(time_obj)
                continue

            # Check for data line
            data_match = re.match(data_pattern, line)
            if data_match:
                timestamp_str = data_match.group(1)
                data_str = data_match.group(2)

                # Parse timestamp
                time_obj = datetime.strptime(timestamp_str, "%Y-%m-%dT%H:%M:%S.%fZ")

                # Parse data fields: split by semicolon, then by comma
                values = {}
                fields = data_str.split(";")
                for field in fields:
                    field = field.strip()
                    if not field:
                        continue
                    parts = field.split(",")
                    if len(parts) >= 2:
                        name = parts[0].strip()
                        value = parts[1].strip()
                        # unit = parts[2].strip() if len(parts) > 2 else ''
                        values[name] = value

                # Extract the relevant values
                try:
                    hr = int(values.get("HR", 0))
                    confidence = int(values.get("HR_Conf", 0))
                    skin_contact = int(values.get("SC", 0))
                    activity_class = int(values.get("Activity", 0))

                    data["timestamps"].append(time_obj)
                    data["hr"].append(hr)
                    data["confidence"].append(confidence)
                    data["skin_contact"].append(skin_contact)
                    data["activity_class"].append(activity_class)
                except (ValueError, KeyError):
                    # Skip lines that don't have the expected data
                    continue

    if not data["timestamps"]:
        raise ValueError("No valid heart rate data found in the log file")

    # Convert to relative time (seconds from start)
    start_time = data["timestamps"][0]
    data["relative_time"] = [
        (t - start_time).total_seconds() for t in data["timestamps"]
    ]
    data["button_press_relative"] = [
        (t - start_time).total_seconds() for t in data["button_press_times"]
    ]

    return data


def calculate_moving_average(values, window_size=5):
    """Calculate moving average with given window size."""
    if len(values) < window_size:
        return values

    moving_avg = np.convolve(values, np.ones(window_size) / window_size, mode="valid")
    # Pad the beginning to match original length
    padding = [np.nan] * (window_size - 1)
    return padding + list(moving_avg)


def calculate_statistics(data):
    """Calculate and return statistics about the data."""
    hr_array = np.array(data["hr"])
    conf_array = np.array(data["confidence"])

    stats = {
        "mean_hr": np.mean(hr_array),
        "min_hr": np.min(hr_array),
        "max_hr": np.max(hr_array),
        "std_hr": np.std(hr_array),
        "mean_conf": np.mean(conf_array),
        "duration": data["relative_time"][-1] if data["relative_time"] else 0,
    }

    # Calculate time in different confidence zones
    conf_high = sum(1 for c in data["confidence"] if c >= 70)
    conf_medium = sum(1 for c in data["confidence"] if 30 <= c < 70)
    conf_low = sum(1 for c in data["confidence"] if c < 30)
    total = len(data["confidence"])

    stats["conf_high_pct"] = (conf_high / total * 100) if total > 0 else 0
    stats["conf_medium_pct"] = (conf_medium / total * 100) if total > 0 else 0
    stats["conf_low_pct"] = (conf_low / total * 100) if total > 0 else 0

    return stats


def plot_data(data):
    """Create interactive plots for the heart rate data."""

    # Calculate moving average for HR
    hr_ma = calculate_moving_average(data["hr"], window_size=5)

    # Calculate statistics
    stats = calculate_statistics(data)

    # Create figure with 3 subplots with custom height ratios
    # HR gets 50% of height, Skin Contact and Activity Class each get 25%
    fig = plt.figure(figsize=(14, 12))
    gs = fig.add_gridspec(3, 1, height_ratios=[2, 1, 1], hspace=0.15)
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    fig.suptitle("Heart Rate Sensor Data Analysis", fontsize=16, fontweight="bold")

    # Convert relative time to datetime objects for better x-axis formatting
    start_time = data["timestamps"][0]
    plot_times = [start_time + timedelta(seconds=t) for t in data["relative_time"]]

    # Identify segments where skin contact != 2 (not skin contact)
    poor_contact_mask = np.array(data["skin_contact"]) != 2

    # Plot 1: Heart Rate and Confidence
    # HR on left axis
    color_hr = "tab:red"
    ax1.set_ylabel("Heart Rate (bpm)", color=color_hr, fontsize=12, fontweight="bold")

    # Plot HR with different colors based on skin contact
    good_contact_hr = [
        hr if not poor else np.nan for hr, poor in zip(data["hr"], poor_contact_mask)
    ]
    poor_contact_hr = [
        hr if poor else np.nan for hr, poor in zip(data["hr"], poor_contact_mask)
    ]

    line1 = ax1.plot(
        plot_times,
        good_contact_hr,
        "o-",
        color=color_hr,
        label="HR (Good Contact)",
        linewidth=1.5,
        markersize=4,
        alpha=0.8,
    )
    line2 = ax1.plot(
        plot_times,
        poor_contact_hr,
        "o-",
        color="orange",
        label="HR (Poor Contact)",
        linewidth=1.5,
        markersize=4,
        alpha=0.8,
    )

    # Plot moving average
    line3 = ax1.plot(
        plot_times,
        hr_ma,
        "--",
        color="darkred",
        label="HR Moving Avg (5 samples)",
        linewidth=2,
        alpha=0.7,
    )

    ax1.tick_params(axis="y", labelcolor=color_hr)
    ax1.grid(True, alpha=0.3)

    # Confidence on right axis
    ax1_right = ax1.twinx()
    color_conf = "tab:blue"
    ax1_right.set_ylabel(
        "Confidence (%)", color=color_conf, fontsize=12, fontweight="bold"
    )
    line4 = ax1_right.plot(
        plot_times,
        data["confidence"],
        "s-",
        color=color_conf,
        label="Confidence",
        linewidth=1.5,
        markersize=3,
        alpha=0.6,
    )
    ax1_right.tick_params(axis="y", labelcolor=color_conf)
    ax1_right.set_ylim(-5, 105)

    # Add confidence zones as background
    ax1_right.axhspan(70, 100, alpha=0.1, color="green", label="High Confidence")
    ax1_right.axhspan(30, 70, alpha=0.1, color="yellow", label="Medium Confidence")
    ax1_right.axhspan(0, 30, alpha=0.1, color="red", label="Low Confidence")

    # Combine legends
    lines = line1 + line2 + line3 + line4
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc="upper left", fontsize=9)

    ax1.set_title("Heart Rate & Confidence over Time", fontsize=13, fontweight="bold")

    # Add vertical lines for button presses on both plots
    if data["button_press_times"]:
        button_plot_times = [
            start_time + timedelta(seconds=t) for t in data["button_press_relative"]
        ]
        for btn_time in button_plot_times:
            ax1.axvline(
                x=btn_time,
                color="purple",
                linestyle="--",
                linewidth=2,
                alpha=0.7,
                label="_nolegend_",
            )
            ax2.axvline(
                x=btn_time,
                color="purple",
                linestyle="--",
                linewidth=2,
                alpha=0.7,
                label="_nolegend_",
            )

        # Add button press to legend (only once)
        ax1.axvline(
            x=button_plot_times[0],
            color="purple",
            linestyle="--",
            linewidth=2,
            alpha=0.7,
            label="Button Press",
        )
        # Update legend
        lines = line1 + line2 + line3 + line4 + [ax1.get_lines()[-1]]
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc="upper left", fontsize=9)

    # Plot 2: Skin Contact
    # Color mapping: 0=red (none), 1=orange (contact not skin), 2=green (skin contact)
    color_map_sc = {0: "red", 1: "orange", 2: "green"}
    colors_sc = [color_map_sc.get(sc, "gray") for sc in data["skin_contact"]]
    ax2.scatter(
        plot_times,
        data["skin_contact"],
        c=colors_sc,
        s=50,
        alpha=0.6,
        edgecolors="black",
        linewidth=0.5,
    )
    ax2.plot(
        plot_times, data["skin_contact"], "-", color="gray", alpha=0.3, linewidth=1
    )

    ax2.set_ylabel("Skin Contact Level", fontsize=12, fontweight="bold")
    ax2.set_xlabel("Time", fontsize=12, fontweight="bold")
    ax2.set_ylim(-0.5, 2.5)
    ax2.set_yticks([0, 1, 2])
    ax2.set_yticklabels(["None", "Contact\n(not skin)", "Skin\nContact"])
    ax2.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax2.grid(True, alpha=0.3)
    ax2.set_title("Skin Contact Status", fontsize=13, fontweight="bold")

    # Add legend for skin contact
    from matplotlib.patches import Patch

    legend_elements = [
        Patch(facecolor="green", alpha=0.6, label="Skin Contact (2)"),
        Patch(facecolor="orange", alpha=0.6, label="Contact, not skin (1)"),
        Patch(facecolor="red", alpha=0.6, label="None (0)"),
    ]

    # Add button press indicator to skin contact legend if present
    if data["button_press_times"]:
        from matplotlib.lines import Line2D

        legend_elements.append(
            Line2D(
                [0],
                [0],
                color="purple",
                linestyle="--",
                linewidth=2,
                alpha=0.7,
                label="Button Press",
            )
        )

    ax2.legend(handles=legend_elements, loc="upper left", fontsize=9)

    # Plot 3: Activity Class
    # Filter out None and 0xFFFFFFFF (4294967295) values which are invalid
    # Note: 0 is a valid activity class value (e.g., resting/no activity)
    valid_ac_mask = [
        (ac is not None and ac != 4294967295) for ac in data["activity_class"]
    ]
    valid_ac_times = [t for t, valid in zip(plot_times, valid_ac_mask) if valid]
    valid_ac_values = [
        ac for ac, valid in zip(data["activity_class"], valid_ac_mask) if valid
    ]

    if valid_ac_values:
        # Define activity labels based on max32664c_algo_activity enum
        activity_labels = {0: "Rest", 1: "Other", 2: "Walk", 3: "Run", 4: "Bike"}

        # Color map for different activities
        activity_colors = {
            0: "blue",  # Rest
            1: "gray",  # Other
            2: "green",  # Walk
            3: "orange",  # Run
            4: "red",  # Bike
        }

        colors = [activity_colors.get(ac, "purple") for ac in valid_ac_values]

        ax3.scatter(
            valid_ac_times,
            valid_ac_values,
            c=colors,
            s=100,
            alpha=0.8,
            edgecolors="black",
            linewidth=1,
            zorder=3,
        )
        ax3.plot(
            valid_ac_times,
            valid_ac_values,
            "-",
            color="gray",
            alpha=0.3,
            linewidth=1.5,
            zorder=2,
        )

        ax3.set_ylabel("Activity Class", fontsize=12, fontweight="bold")
        ax3.set_xlabel("Time", fontsize=12, fontweight="bold")
        ax3.set_ylim(-0.5, 4.5)
        ax3.set_yticks([0, 1, 2, 3, 4])
        ax3.set_yticklabels([activity_labels.get(i, str(i)) for i in range(5)])
        ax3.grid(True, alpha=0.3)
        ax3.set_title("Activity Classification", fontsize=13, fontweight="bold")

        # Create legend with activity types
        from matplotlib.patches import Patch

        legend_elements = []
        unique_activities = sorted(set(valid_ac_values))
        for act in unique_activities:
            legend_elements.append(
                Patch(
                    facecolor=activity_colors.get(act, "purple"),
                    alpha=0.8,
                    edgecolor="black",
                    linewidth=1,
                    label=f'{activity_labels.get(act, "Unknown")} ({act})',
                )
            )
        ax3.legend(handles=legend_elements, loc="upper left", fontsize=9)

        # Add button press vertical lines to activity class plot
        if data["button_press_times"]:
            for btn_time in button_plot_times:
                ax3.axvline(
                    x=btn_time,
                    color="purple",
                    linestyle="--",
                    linewidth=2,
                    alpha=0.7,
                    label="_nolegend_",
                )
    else:
        # No valid activity class data, show message
        ax3.text(
            0.5,
            0.5,
            "No valid Activity Class data\n(all values are None or 0xFFFFFFFF)",
            transform=ax3.transAxes,
            fontsize=12,
            ha="center",
            va="center",
            bbox=dict(boxstyle="round", facecolor="lightgray", alpha=0.8),
        )
        ax3.set_ylabel("Activity Class", fontsize=12, fontweight="bold")
        ax3.set_xlabel("Time", fontsize=12, fontweight="bold")
        ax3.set_title("Activity Classification", fontsize=13, fontweight="bold")
        ax3.grid(True, alpha=0.3)

    # Format x-axis to show time nicely
    ax3.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    plt.setp(ax3.xaxis.get_majorticklabels(), rotation=45, ha="right")

    # Add statistics text box
    stats_text = (
        f"Statistics:\n"
        f"Duration: {stats['duration']:.1f}s\n"
        f"Mean HR: {stats['mean_hr']:.1f} bpm\n"
        f"HR Range: {stats['min_hr']}-{stats['max_hr']} bpm\n"
        f"Std Dev: {stats['std_hr']:.1f} bpm\n"
        f"Mean Confidence: {stats['mean_conf']:.1f}%\n"
        f"High Conf: {stats['conf_high_pct']:.1f}%\n"
        f"Med Conf: {stats['conf_medium_pct']:.1f}%\n"
        f"Low Conf: {stats['conf_low_pct']:.1f}%"
    )

    # Add text box with stats
    props = dict(boxstyle="round", facecolor="wheat", alpha=0.8)
    ax1.text(
        0.98,
        0.97,
        stats_text,
        transform=ax1.transAxes,
        fontsize=9,
        verticalalignment="top",
        horizontalalignment="right",
        bbox=props,
        family="monospace",
    )

    # Adjust layout
    plt.subplots_adjust(top=0.96, bottom=0.08)
    plt.show()


def main():
    """Main function to run the plotter."""
    if len(sys.argv) != 2:
        print("Usage: python hr_plotter.py <log_file_path>")
        print("Example: python hr_plotter.py sample_log.txt")
        sys.exit(1)

    log_file = sys.argv[1]

    try:
        print(f"Parsing log file: {log_file}")
        data = parse_log_file(log_file)
        print(f"Found {len(data['hr'])} data points")
        print(
            f"Time range: {data['timestamps'][0].strftime('%Y-%m-%d %H:%M:%S')} - {data['timestamps'][-1].strftime('%Y-%m-%d %H:%M:%S')}"
        )

        # Check for valid activity class data
        valid_ac = [
            ac for ac in data["activity_class"] if ac is not None and ac != 4294967295
        ]
        if valid_ac:
            print(f"Found {len(valid_ac)} valid Activity Class data points")
        else:
            print("No valid Activity Class data (all values are None or 0xFFFFFFFF)")

        if data["button_press_times"]:
            print(f"Found {len(data['button_press_times'])} button press event(s)")
            for i, btn_time in enumerate(data["button_press_times"], 1):
                print(
                    f"  Button press {i}: {btn_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}"
                )

        print("\nGenerating plots...")
        plot_data(data)

    except FileNotFoundError:
        print(f"Error: File '{log_file}' not found")
        sys.exit(1)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
