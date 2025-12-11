#!/usr/bin/env python3
"""
Real-time plotter for heart rate sensor raw data (PPG and accelerometer).
Streams data via RTT and renders with PyQtGraph for high-frequency updates.
"""

import csv
import logging
import sys
import time
from typing import Dict, Optional

import numpy as np
import pylink
from scipy import signal

try:
    from PyQt5 import QtCore, QtWidgets  # type: ignore
    QT_API = "PyQt5"
except ImportError:  # pragma: no cover - runtime fallback only
    from PySide6 import QtCore, QtWidgets  # type: ignore
    QT_API = "PySide6"

import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter

from utils import current_milli_time

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)
log = logging.getLogger(__name__)

# Configuration
TARGET_DEVICE = "NRF54L15_M33"
SERIAL_NUMBER = None
# Buffer size (fixed for numpy arrays). Should be >= WINDOW_SIZE * effective_sample_rate.
MAX_POINTS = 40000
WINDOW_SIZE = 20.0  # Display window in seconds (scrolls within MAX_POINTS)
SAMPLE_RATE = 100  # Expected Hz, for preallocating (informational)
# Max points to draw per line after downsampling (keeps draw fast regardless of input rate)
DRAW_MAX_POINTS = 2000
# Periodic autoscale for PPG subplot to keep signal visible (seconds)
PPG_AUTOSCALE_EVERY_S = 1.0
PPG_Y_PADDING_FRACTION = 0.10  # 10% padding around min/max
PPG_MIN_SPAN = 100.0  # ensure at least this span when autoscaling

# Fixed axis ranges
PPG_Y_MIN = 0
PPG_Y_MAX = 2000000
ACC_Y_MIN = -1034
ACC_Y_MAX = 1034

# PSD Configuration
PSD_WINDOW_LENGTH = 8.0  # seconds per Welch segment
PSD_OVERLAP = 0.5  # 50% overlap
PSD_UPDATE_INTERVAL = 0.5  # Update PSD every 0.5 seconds (2 Hz)
PSD_FREQ_MIN = 0.0  # Hz
PSD_FREQ_MAX = 5.0  # Hz
PSD_ANALYSIS_MIN_HZ = 0.3  # Hz (physiological lower bound)
PSD_ANALYSIS_MAX_HZ = 5.0  # Hz (physiological upper bound)
PSD_PEAK_TOLERANCE_HZ = 0.12  # Hz half-bandwidth around dominant peak
PSD_HIGHPASS_CUTOFF_HZ = 0.2  # Light high-pass for PSD-only detrending
EMBEDDED_HR_CONF_THRESHOLD = 70.0  # % threshold for mismatch diagnostics
PSD_Y_MIN = -60  # dB/Hz
PSD_Y_MAX = 40  # dB/Hz

# Preallocated NumPy buffers for speed - no Python lists/deques in hot path
buffers = {
    "GREEN1": np.zeros(MAX_POINTS, dtype=np.float32),
    "IR1": np.zeros(MAX_POINTS, dtype=np.float32),
    "RED1": np.zeros(MAX_POINTS, dtype=np.float32),
    "GREEN2": np.zeros(MAX_POINTS, dtype=np.float32),
    "IR2": np.zeros(MAX_POINTS, dtype=np.float32),
    "RED2": np.zeros(MAX_POINTS, dtype=np.float32),
    "X": np.zeros(MAX_POINTS, dtype=np.float32),
    "Y": np.zeros(MAX_POINTS, dtype=np.float32),
    "Z": np.zeros(MAX_POINTS, dtype=np.float32),
    "HR": np.zeros(MAX_POINTS, dtype=np.float32),
    "HR_Conf": np.zeros(MAX_POINTS, dtype=np.float32),
    "RR": np.zeros(MAX_POINTS, dtype=np.float32),
    "RR_Conf": np.zeros(MAX_POINTS, dtype=np.float32),
    "SC": np.zeros(MAX_POINTS, dtype=np.float32),
    "Activity": np.zeros(MAX_POINTS, dtype=np.float32),
    "SpO2": np.zeros(MAX_POINTS, dtype=np.float32),
    "SpO2_Conf": np.zeros(MAX_POINTS, dtype=np.float32),
    "time": np.zeros(MAX_POINTS, dtype=np.float32),
}

# Global variables
jlink = None
start_time = None
ts_start_ms: Optional[float] = None
last_elapsed_time = 0.0
line_buffer = ""
sample_count = 0
write_idx = 0  # Circular buffer index


def parse_data_line(line: str) -> Optional[Dict[str, int]]:
    """
    Parse a line of data in the format:
    GREEN1,178996,;GREEN2,174866,;IR1,524287,;IR2,524287,;RED1,524287,;RED2,524287,;X,449,;Y,248,;Z,856;HR,97,bpm;HR_Conf,0,;RR,0,ms;RR_Conf,0,;SC,1,;Activity,0,;SpO2,0,%;SpO2_Conf,6291456;

    Returns:
        dict with parsed integer values when successful, otherwise None.
    """
    data: Dict[str, int] = {}
    try:
        pairs = line.strip().rstrip(";").split(";")
        for pair in pairs:
            if "," not in pair:
                continue
            parts = pair.split(",")
            if len(parts) < 2:
                continue
            key = parts[0].strip()
            value = parts[1].strip()
            try:
                data[key] = int(value)
                if key == "GREEN1" and int(value) < 20:
                    print("Invalid GREEN1 value detected:", line)
            except ValueError:
                # Skip non-integer values (e.g. units like "bpm", "ms", "%")
                continue

        expected_keys = ["GREEN1", "GREEN2", "IR1", "IR2", "RED1", "RED2", "X", "Y", "Z"]
        if all(key in data for key in expected_keys):
            return data
    except (ValueError, AttributeError) as exc:
        log.debug("Failed to parse line %s - %s", line.strip(), exc)

    return None


def init_jlink() -> None:
    """Initialize JLink connection and RTT."""
    global jlink, start_time

    jlink = pylink.JLink()
    logging.getLogger("pylink.jlink").setLevel(logging.WARNING)

    log.info("Connecting to JLink using %s...", QT_API)
    if SERIAL_NUMBER:
        jlink.open(serial_no=SERIAL_NUMBER)
    else:
        jlink.open()

    log.info("Connecting to %s...", TARGET_DEVICE)
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(TARGET_DEVICE)
    jlink.rtt_start(None)

    start_time = current_milli_time()
    log.info("RTT connection established. Starting data acquisition...")


def read_rtt_data() -> int:
    """Read data from RTT and parse it into the circular buffer."""
    global line_buffer, sample_count, write_idx, ts_start_ms, last_elapsed_time

    if not jlink or not jlink.connected():
        return 0

    new_samples = 0
    try:
        read_bytes = jlink.rtt_read(0, 4096)
        if not read_bytes:
            return 0

        data = "".join(map(chr, read_bytes))
        line_buffer += data

        lines = line_buffer.split("\n")
        line_buffer = lines[-1]  # keep incomplete line

        for line in lines[:-1]:
            parsed_data = parse_data_line(line)
            if not parsed_data:
                continue

            if "TS" in parsed_data:
                ts_value = float(parsed_data["TS"])
                if ts_start_ms is None:
                    ts_start_ms = ts_value
                elapsed_time = max(0.0, (ts_value - ts_start_ms) / 1000.0)
            else:
                elapsed_time = (current_milli_time() - start_time) / 1000.0
            if write_idx > 0:
                prev_time = last_elapsed_time
                if elapsed_time <= prev_time:
                    elapsed_time = prev_time + (1.0 / max(1.0, SAMPLE_RATE))
            last_elapsed_time = elapsed_time
            idx = write_idx % MAX_POINTS
            buffers["time"][idx] = elapsed_time
            buffers["GREEN1"][idx] = parsed_data["GREEN1"]
            buffers["IR1"][idx] = parsed_data["IR1"]
            buffers["RED1"][idx] = parsed_data["RED1"]
            buffers["GREEN2"][idx] = parsed_data["GREEN2"]
            buffers["IR2"][idx] = parsed_data["IR2"]
            buffers["RED2"][idx] = parsed_data["RED2"]
            buffers["X"][idx] = parsed_data["X"]
            buffers["Y"][idx] = parsed_data["Y"]
            buffers["Z"][idx] = parsed_data["Z"]
            
            # Only update HR/confidence fields if present (carry forward previous values)
            if "HR" in parsed_data:
                buffers["HR"][idx] = parsed_data["HR"]
            elif write_idx > 0:
                prev_idx = (write_idx - 1) % MAX_POINTS
                buffers["HR"][idx] = buffers["HR"][prev_idx]
            
            if "HR_Conf" in parsed_data:
                buffers["HR_Conf"][idx] = parsed_data["HR_Conf"]
            elif write_idx > 0:
                prev_idx = (write_idx - 1) % MAX_POINTS
                buffers["HR_Conf"][idx] = buffers["HR_Conf"][prev_idx]
            
            if "RR" in parsed_data:
                buffers["RR"][idx] = parsed_data["RR"]
            elif write_idx > 0:
                prev_idx = (write_idx - 1) % MAX_POINTS
                buffers["RR"][idx] = buffers["RR"][prev_idx]
            
            if "RR_Conf" in parsed_data:
                buffers["RR_Conf"][idx] = parsed_data["RR_Conf"]
            elif write_idx > 0:
                prev_idx = (write_idx - 1) % MAX_POINTS
                buffers["RR_Conf"][idx] = buffers["RR_Conf"][prev_idx]
            
            if "SC" in parsed_data:
                buffers["SC"][idx] = parsed_data["SC"]
            elif write_idx > 0:
                prev_idx = (write_idx - 1) % MAX_POINTS
                buffers["SC"][idx] = buffers["SC"][prev_idx]
            
            if "Activity" in parsed_data:
                buffers["Activity"][idx] = parsed_data["Activity"]
            elif write_idx > 0:
                prev_idx = (write_idx - 1) % MAX_POINTS
                buffers["Activity"][idx] = buffers["Activity"][prev_idx]
            
            if "SpO2" in parsed_data:
                buffers["SpO2"][idx] = parsed_data["SpO2"]
            elif write_idx > 0:
                prev_idx = (write_idx - 1) % MAX_POINTS
                buffers["SpO2"][idx] = buffers["SpO2"][prev_idx]
            
            if "SpO2_Conf" in parsed_data:
                buffers["SpO2_Conf"][idx] = parsed_data["SpO2_Conf"]
            elif write_idx > 0:
                prev_idx = (write_idx - 1) % MAX_POINTS
                buffers["SpO2_Conf"][idx] = buffers["SpO2_Conf"][prev_idx]

            write_idx += 1
            new_samples += 1
            sample_count += 1

        if new_samples and sample_count % 200 == 0:
            log.info("Samples: %s, write idx: %s", sample_count, write_idx)

    except Exception as exc:  # pragma: no cover - defensive logging
        log.error("Error reading RTT data: %s", exc)

    return new_samples


def clear_stream_buffers() -> None:
    """Reset circular buffers and associated counters."""
    global line_buffer, sample_count, write_idx, start_time, ts_start_ms, last_elapsed_time
    for array in buffers.values():
        array.fill(0)
    line_buffer = ""
    sample_count = 0
    write_idx = 0
    start_time = current_milli_time()
    ts_start_ms = None
    last_elapsed_time = 0.0
    log.info("Cleared buffered samples.")


def _ordered_view(array: np.ndarray, num_samples: int) -> np.ndarray:
    """Return a chronological view of the circular buffer contents."""
    if write_idx < MAX_POINTS:
        return array[:num_samples]

    start_idx = write_idx % MAX_POINTS
    return np.concatenate([array[start_idx:], array[:start_idx]])


def extract_windowed_data() -> Optional[Dict[str, np.ndarray]]:
    """Return the most recent WINDOW_SIZE seconds of samples."""
    if write_idx == 0:
        return None

    num_samples = min(write_idx, MAX_POINTS)
    if num_samples < 2:
        return None

    time_view = _ordered_view(buffers["time"], num_samples)
    ppg1_view = _ordered_view(buffers["GREEN1"], num_samples)
    ppg2_view = _ordered_view(buffers["GREEN2"], num_samples)
    accx_view = _ordered_view(buffers["X"], num_samples)
    accy_view = _ordered_view(buffers["Y"], num_samples)
    accz_view = _ordered_view(buffers["Z"], num_samples)
    hr_view = _ordered_view(buffers["HR"], num_samples)
    hr_conf_view = _ordered_view(buffers["HR_Conf"], num_samples)

    current_time = time_view[-1]
    if current_time < WINDOW_SIZE:
        time_display = time_view
        ppg1_display = ppg1_view
        ppg2_display = ppg2_view
        accx_display = accx_view
        accy_display = accy_view
        accz_display = accz_view
        hr_display = hr_view
        hr_conf_display = hr_conf_view
    else:
        window_start = current_time - WINDOW_SIZE
        mask = time_view >= window_start
        time_display = time_view[mask] - window_start
        ppg1_display = ppg1_view[mask]
        ppg2_display = ppg2_view[mask]
        accx_display = accx_view[mask]
        accy_display = accy_view[mask]
        accz_display = accz_view[mask]
        hr_display = hr_view[mask]
        hr_conf_display = hr_conf_view[mask]

    if time_view[-1] > 0:
        effective_rate = num_samples / time_view[-1]
        needed = int(WINDOW_SIZE * max(1.0, effective_rate))
        if needed > MAX_POINTS and (sample_count % 500 == 0):
            log.warning(
                "MAX_POINTS=%s may be too small for WINDOW_SIZE=%ss at ~%s Hz. "
                "Consider increasing to >= %s",
                MAX_POINTS,
                WINDOW_SIZE,
                f"{effective_rate:.0f}",
                needed,
            )

    return {
        "time": np.asarray(time_display),
        "GREEN1": np.asarray(ppg1_display),
        "GREEN2": np.asarray(ppg2_display),
        "X": np.asarray(accx_display),
        "Y": np.asarray(accy_display),
        "Z": np.asarray(accz_display),
        "HR": np.asarray(hr_display),
        "HR_Conf": np.asarray(hr_conf_display),
        "time_full": np.asarray(time_display),  # Keep original time for HR filtering
    }


def decimate(x: np.ndarray, y: np.ndarray, nmax: int) -> tuple[np.ndarray, np.ndarray]:
    """Thin data for plotting without heavy resampling."""
    if x.size <= nmax:
        return x, y
    step = max(1, x.size // nmax)
    return x[::step], y[::step]


def compute_psd(
    time_data: np.ndarray,
    signal_data: np.ndarray,
    sample_rate: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """
    Compute Power Spectral Density using Welch's method.

    Args:
        time_data: Time array (retained for API symmetry; unused)
        signal_data: Signal values
        sample_rate: Effective sample rate in Hz

    Returns:
        (frequencies, psd_linear, psd_db, nperseg)
    """
    if signal_data.size < 100 or sample_rate <= 0.0:
        return np.array([]), np.array([]), np.array([]), 0

    # Detrend by removing the mean, then apply a light high-pass for PSD only
    signal_detrended = signal_data - np.mean(signal_data)

    if signal_detrended.size > 3:
        nyquist = 0.5 * sample_rate
        cutoff = min(PSD_HIGHPASS_CUTOFF_HZ, nyquist * 0.99)
        if 0 < cutoff < nyquist:
            norm_cut = cutoff / nyquist
            try:
                sos = signal.butter(
                    N=2,
                    Wn=norm_cut,
                    btype="highpass",
                    output="sos",
                )
                padlen = min(250, signal_detrended.size - 1)
                if padlen >= 1:
                    signal_detrended = signal.sosfiltfilt(sos, signal_detrended, padlen=padlen)
                else:
                    signal_detrended = signal.sosfilt(sos, signal_detrended)
            except ValueError:
                pass  # Fallback to mean-only detrending if filter fails

    # Welch segment sizing: target ~PSD_WINDOW_LENGTH seconds, but ensure >= 4*fs when possible
    target_nperseg = int(PSD_WINDOW_LENGTH * sample_rate)
    min_required = int(4 * sample_rate) if sample_rate > 0 else 0
    nperseg = max(64, target_nperseg, min_required)
    nperseg = min(signal_detrended.size, nperseg)
    if nperseg <= 1:
        return np.array([]), np.array([]), np.array([]), 0

    try:
        freqs, psd_linear = signal.welch(
            signal_detrended,
            fs=sample_rate,
            window="hann",
            nperseg=nperseg,
            noverlap=int(nperseg * PSD_OVERLAP),
            scaling="density",
        )
        psd_db = 10 * np.log10(psd_linear + 1e-12)
        return freqs, psd_linear, psd_db, nperseg
    except Exception as exc:  # pragma: no cover - defensive logging
        log.warning("PSD computation failed: %s", exc)
        return np.array([]), np.array([]), np.array([]), 0


def compute_psd_metrics(
    freqs: np.ndarray,
    psd_linear: np.ndarray,
    psd_db: np.ndarray,
    sample_rate: float,
    nperseg: int,
    embedded_hr_bpm: float,
    embedded_hr_conf: float,
    use_embedded_reference: bool = False,
) -> Dict[str, float | str | bool]:
    """
    Derive hardware and comparison metrics from a PSD estimate.
    """
    metrics: Dict[str, float | str | bool] = {
        "peak_freq_hz": float("nan"),
        "peak_bpm": float("nan"),
        "peak_power_db": float("nan"),
        "peak_prom_db": float("nan"),
        "peak_width_hz": float("nan"),
        "snr_db": float("nan"),
        "noise_floor_db": float("nan"),
        "signal_bandwidth_hz": float("nan"),
        "center_freq_hz": float("nan"),
        "center_label": "peak",
        "freq_tolerance_hz": float("nan"),
        "delta_f_hz": float("nan"),
        "fs_hz": float(sample_rate),
        "nperseg": float(nperseg),
        "embedded_hr_bpm": float(embedded_hr_bpm),
        "embedded_hr_conf": float(embedded_hr_conf),
        "mismatch": False,
        "mismatch_reason": "Embedded HR unavailable",
        "mismatch_bpm_diff": float("nan"),
        "tolerance_bpm": float("nan"),
        "peak_power_linear": float("nan"),
        "noise_density_linear": float("nan"),
    }

    if freqs.size == 0 or psd_linear.size == 0 or psd_db.size == 0:
        return metrics

    band_mask = (freqs >= PSD_ANALYSIS_MIN_HZ) & (freqs <= PSD_ANALYSIS_MAX_HZ)
    if not np.any(band_mask):
        return metrics

    freqs_band = freqs[band_mask]
    psd_linear_band = psd_linear[band_mask]
    psd_db_band = psd_db[band_mask]

    if freqs_band.size == 0:
        return metrics

    df = float(np.median(np.diff(freqs_band))) if freqs_band.size > 1 else float("nan")
    metrics["delta_f_hz"] = df

    peak_idx = int(np.argmax(psd_linear_band))
    peak_freq_hz = float(freqs_band[peak_idx])
    peak_bpm = peak_freq_hz * 60.0
    peak_power_linear = float(psd_linear_band[peak_idx])
    peak_power_db = float(10 * np.log10(peak_power_linear + 1e-12))

    metrics["peak_freq_hz"] = peak_freq_hz
    metrics["peak_bpm"] = peak_bpm
    metrics["peak_power_db"] = peak_power_db
    metrics["peak_power_linear"] = peak_power_linear

    # Peak width via contiguous half-power points (FWHM)
    if np.isfinite(peak_power_linear) and peak_power_linear > 0:
        half_power = peak_power_linear / 2.0
        left = peak_idx
        while left > 0 and psd_linear_band[left - 1] >= half_power:
            left -= 1
        right = peak_idx
        while right < psd_linear_band.size - 1 and psd_linear_band[right + 1] >= half_power:
            right += 1
        if right > left:
            metrics["peak_width_hz"] = float(freqs_band[right] - freqs_band[left])

    center_freq_hz = peak_freq_hz
    center_label = "peak"
    if use_embedded_reference and embedded_hr_bpm > 0:
        center_freq_hz = embedded_hr_bpm / 60.0
        center_label = "embedded"
    metrics["center_freq_hz"] = float(center_freq_hz)
    metrics["center_label"] = center_label

    freq_tolerance = float("nan")
    if np.isfinite(df):
        freq_tolerance = max(PSD_PEAK_TOLERANCE_HZ, 2.0 * df)
    elif freqs_band.size > 1:
        freq_tolerance = max(PSD_PEAK_TOLERANCE_HZ, float(freqs_band[1] - freqs_band[0]) * 2.0)
    else:
        freq_tolerance = PSD_PEAK_TOLERANCE_HZ
    metrics["freq_tolerance_hz"] = float(freq_tolerance)

    signal_mask = np.abs(freqs_band - center_freq_hz) <= freq_tolerance
    harmonic_mask = np.abs(freqs_band - (2.0 * center_freq_hz)) <= freq_tolerance

    signal_bandwidth_hz = float(np.sum(signal_mask) * (df if np.isfinite(df) else 0.0))
    metrics["signal_bandwidth_hz"] = signal_bandwidth_hz

    signal_power = float("nan")
    if np.any(signal_mask) and signal_bandwidth_hz > 0:
        signal_power = float(np.sum(psd_linear_band[signal_mask]) * (df if np.isfinite(df) else 0.0))

    noise_mask = ~(signal_mask | harmonic_mask)
    if np.any(noise_mask):
        noise_density = float(np.median(psd_linear_band[noise_mask]))
    else:
        noise_density = float(np.median(psd_linear_band))
    metrics["noise_density_linear"] = noise_density
    metrics["noise_floor_db"] = float(10 * np.log10(noise_density + 1e-12))

    if np.isfinite(peak_power_db) and np.isfinite(metrics["noise_floor_db"]):
        metrics["peak_prom_db"] = float(peak_power_db - metrics["noise_floor_db"])

    if np.isfinite(signal_power) and signal_power > 0 and noise_density > 0 and signal_bandwidth_hz > 0:
        noise_power = noise_density * signal_bandwidth_hz
        snr_linear = signal_power / noise_power if noise_power > 0 else float("inf")
        metrics["snr_db"] = float(10 * np.log10(snr_linear + 1e-12))

    tolerance_hz = float(freq_tolerance)
    metrics["tolerance_bpm"] = float(tolerance_hz * 60.0)

    if embedded_hr_bpm > 0:
        metrics["mismatch_bpm_diff"] = float(abs(peak_bpm - embedded_hr_bpm))
        if embedded_hr_conf >= EMBEDDED_HR_CONF_THRESHOLD:
            if np.isfinite(tolerance_hz):
                diff_hz = abs(peak_freq_hz - embedded_hr_bpm / 60.0)
                if diff_hz > tolerance_hz:
                    metrics["mismatch"] = True
                    metrics[
                        "mismatch_reason"
                    ] = f"|Δ|={metrics['mismatch_bpm_diff']:.1f} bpm > {metrics['tolerance_bpm']:.1f} bpm tolerance"
                else:
                    metrics[
                        "mismatch_reason"
                    ] = f"|Δ|={metrics['mismatch_bpm_diff']:.1f} bpm ≤ {metrics['tolerance_bpm']:.1f} bpm tolerance"
            else:
                metrics["mismatch_reason"] = "Δf undefined; cannot assess mismatch"
        else:
            metrics[
                "mismatch_reason"
            ] = f"HR_Conf {embedded_hr_conf:.0f}% < {EMBEDDED_HR_CONF_THRESHOLD:.0f}% threshold"

    return metrics


class HRPlotWindow(QtWidgets.QMainWindow):
    """PyQtGraph window hosting the live plots."""

    def __init__(self, on_close=None, on_clear=None, on_pause=None):
        super().__init__()
        self._on_close = on_close
        self._on_clear = on_clear
        self._on_pause = on_pause

        self.setWindowTitle("Heart Rate Sensor - Real-time Raw Data with PSD (PyQtGraph)")
        self.resize(1600, 900)

        pg.setConfigOptions(
            antialias=False,
            useOpenGL=True,
            background="w",
            foreground="k",
        )

        central = QtWidgets.QWidget()
        central_layout = QtWidgets.QVBoxLayout(central)
        central_layout.setContentsMargins(6, 6, 6, 6)
        central_layout.setSpacing(6)

        # Status label showing latest values
        self.status_label = QtWidgets.QLabel("Waiting for data...")
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                border: 1px solid #ccc;
                border-radius: 3px;
                padding: 8px;
                font-family: monospace;
                font-size: 11px;
            }
        """)
        self.status_label.setWordWrap(True)
        central_layout.addWidget(self.status_label)

        controls_layout = QtWidgets.QHBoxLayout()
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setSpacing(6)

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.setCheckable(True)
        self.pause_button.clicked.connect(self._handle_pause_clicked)
        self.pause_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                padding: 5px 15px;
                border-radius: 3px;
            }
            QPushButton:checked {
                background-color: #ff9800;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:checked:hover {
                background-color: #e68900;
            }
        """)
        controls_layout.addWidget(self.pause_button)

        self.clear_button = QtWidgets.QPushButton("Clear Data")
        self.clear_button.clicked.connect(self._handle_clear_clicked)
        controls_layout.addWidget(self.clear_button)
        
        # PSD Controls
        psd_label = QtWidgets.QLabel("PSD:")
        controls_layout.addWidget(psd_label)
        
        self.psd_lock_scale_checkbox = QtWidgets.QCheckBox("Lock Y-scale")
        self.psd_lock_scale_checkbox.setChecked(True)
        controls_layout.addWidget(self.psd_lock_scale_checkbox)
        
        self.psd_channel_combo = QtWidgets.QComboBox()
        self.psd_channel_combo.addItems(["GREEN1", "GREEN2"])
        self.psd_channel_combo.setCurrentIndex(0)
        controls_layout.addWidget(self.psd_channel_combo)

        self.use_embedded_checkbox = QtWidgets.QCheckBox("Use embedded HR for metrics")
        self.use_embedded_checkbox.setChecked(False)
        self.use_embedded_checkbox.toggled.connect(self._handle_use_embedded_toggled)
        controls_layout.addWidget(self.use_embedded_checkbox)
        
        self.snapshot_button = QtWidgets.QPushButton("Snapshot")
        self.snapshot_button.clicked.connect(self._handle_snapshot_clicked)
        controls_layout.addWidget(self.snapshot_button)
        
        controls_layout.addStretch()

        central_layout.addLayout(controls_layout)

        self._layout = pg.GraphicsLayoutWidget()
        central_layout.addWidget(self._layout)
        self.setCentralWidget(central)

        self.channel_colors = {
            "GREEN1": "#1f77b4",
            "GREEN2": "#d62728",
        }
        self._latest_psd_metrics: Optional[Dict[str, Dict[str, object]]] = None

        # GREEN1 plot
        self.ppg1_plot = self._layout.addPlot(
            row=0,
            col=0,
            title="GREEN1",
        )
        self._configure_ppg_plot(self.ppg1_plot, "Amplitude")
        self.green1_curve = self.ppg1_plot.plot(pen=pg.mkPen(color="#1f77b4", width=1), name="GREEN1")
        self.green1_curve.setClipToView(True)
        self.green1_curve.setDownsampling(auto=True, method="peak")
        self.green1_text = pg.TextItem(color=(0, 0, 0), anchor=(0, 1))
        self.ppg1_plot.addItem(self.green1_text)

        # GREEN2 plot
        self.ppg2_plot = self._layout.addPlot(
            row=1,
            col=0,
            title="GREEN2",
        )
        self._configure_ppg_plot(self.ppg2_plot, "Amplitude")
        self.green2_curve = self.ppg2_plot.plot(pen=pg.mkPen(color="#d62728", width=1), name="GREEN2")
        self.green2_curve.setClipToView(True)
        self.green2_curve.setDownsampling(auto=True, method="peak")
        self.green2_text = pg.TextItem(color=(0, 0, 0), anchor=(0, 1))
        self.ppg2_plot.addItem(self.green2_text)

        # HR and HR_Conf plot (dual y-axis)
        self.hr_plot = self._layout.addPlot(
            row=2,
            col=0,
            title="Heart Rate & Confidence",
        )
        self.hr_plot.showGrid(x=True, y=True, alpha=0.3)
        self.hr_plot.setLabel("left", "HR (bpm)", color="#1f77b4")
        self.hr_plot.setLabel("bottom", "Time (s)")
        self.hr_plot.setXRange(0, WINDOW_SIZE, padding=0)
        self.hr_plot.setYRange(40, 200)
        self.hr_plot.setLimits(xMin=0, yMin=0, yMax=250)
        self.hr_plot.setMouseEnabled(x=False, y=False)
        self.hr_plot.addLegend(offset=(10, 10))
        self.hr_plot.legend.setBrush(pg.mkBrush(255, 255, 255, 180))
        
        # HR curve on left axis
        self.hr_curve = self.hr_plot.plot(
            pen=pg.mkPen(color="#1f77b4", width=2),
            name="HR",
        )
        self.hr_curve.setClipToView(True)
        
        # Create right axis for HR_Conf
        self.hr_conf_axis = pg.ViewBox()
        self.hr_plot.showAxis("right")
        self.hr_plot.scene().addItem(self.hr_conf_axis)
        self.hr_plot.getAxis("right").linkToView(self.hr_conf_axis)
        self.hr_conf_axis.setXLink(self.hr_plot)
        self.hr_plot.getAxis("right").setLabel("HR Confidence (%)", color="#ff7f0e")
        
        # Set fixed range for HR_Conf: 0-100%
        self.hr_conf_axis.setYRange(0, 100, padding=0)
        
        # HR_Conf scatter plot (dots only) on right axis
        self.hr_conf_scatter = pg.ScatterPlotItem(
            size=6,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(255, 127, 14, 200),
        )
        self.hr_conf_axis.addItem(self.hr_conf_scatter)
        
        # Update views when plot is resized
        def update_hr_conf_views():
            self.hr_conf_axis.setGeometry(self.hr_plot.vb.sceneBoundingRect())
            self.hr_conf_axis.linkedViewChanged(self.hr_plot.vb, self.hr_conf_axis.XAxis)
        
        update_hr_conf_views()
        self.hr_plot.vb.sigResized.connect(update_hr_conf_views)

        # Accelerometer plot
        self.acc_plot = self._layout.addPlot(
            row=3,
            col=0,
            title="Accelerometer (X, Y, Z)",
        )
        self.acc_plot.showGrid(x=True, y=True, alpha=0.3)
        self.acc_plot.setLabel("left", "Acceleration (mg)")
        self.acc_plot.setLabel("bottom", "Time (s)")
        self.acc_plot.setXRange(0, WINDOW_SIZE, padding=0)
        self.acc_plot.setYRange(ACC_Y_MIN, ACC_Y_MAX)
        self.acc_plot.setLimits(xMin=0, yMin=ACC_Y_MIN * 5, yMax=ACC_Y_MAX * 5)
        self.acc_plot.setMouseEnabled(x=False, y=False)
        self.acc_plot.addLegend(offset=(10, 10))
        self.acc_plot.legend.setBrush(pg.mkBrush(255, 255, 255, 180))
        self.acc_curve_x = self.acc_plot.plot(
            pen=pg.mkPen(color="#e377c2", width=1),
            name="X",
        )
        self.acc_curve_y = self.acc_plot.plot(
            pen=pg.mkPen(color="#2ca02c", width=1),
            name="Y",
        )
        self.acc_curve_z = self.acc_plot.plot(
            pen=pg.mkPen(color="#1f77b4", width=1),
            name="Z",
        )
        for curve in (self.acc_curve_x, self.acc_curve_y, self.acc_curve_z):
            curve.setClipToView(True)
            curve.setDownsampling(auto=True, method="peak")

        # PSD plot
        self.psd_plot = self._layout.addPlot(
            row=0,
            col=1,
            rowspan=4,
            title="PSD 0–5 Hz",
        )
        self.psd_plot.showGrid(x=True, y=True, alpha=0.3)
        self.psd_plot.setLabel("left", "Power (dB/Hz)")
        self.psd_plot.setLabel("bottom", "Frequency (Hz)")
        self.psd_plot.setXRange(PSD_FREQ_MIN, PSD_FREQ_MAX, padding=0)
        self.psd_plot.setYRange(PSD_Y_MIN, PSD_Y_MAX)
        self.psd_plot.setLimits(xMin=0, xMax=6, yMin=-80, yMax=60)
        self.psd_plot.setMouseEnabled(x=False, y=True)
        self.psd_plot.addLegend(offset=(10, 10))
        self.psd_plot.legend.setBrush(pg.mkBrush(255, 255, 255, 180))
        
        # PSD curves for GREEN1 and GREEN2
        self.psd_curve_green1 = self.psd_plot.plot(
            pen=pg.mkPen(color="#1f77b4", width=2),
            name="GREEN1",
        )
        self.psd_curve_green2 = self.psd_plot.plot(
            pen=pg.mkPen(color="#d62728", width=2),
            name="GREEN2",
        )

        # PSD peak marker (solid)
        self.psd_peak_line = pg.InfiniteLine(
            angle=90,
            movable=False,
            pen=pg.mkPen(color="#1f77b4", width=2),
            label="PSD Peak",
        )
        self.psd_plot.addItem(self.psd_peak_line)
        self.psd_peak_line.setVisible(False)

        # Embedded HR marker (dashed)
        self.psd_embedded_line = pg.InfiniteLine(
            angle=90,
            movable=False,
            pen=pg.mkPen(color="#2ca02c", width=2, style=QtCore.Qt.PenStyle.DashLine),
            label="Embedded HR",
        )
        self.psd_plot.addItem(self.psd_embedded_line)
        self.psd_embedded_line.setVisible(False)

        # Embedded HR tolerance region (shaded)
        self.psd_embedded_region = pg.LinearRegionItem(
            values=[0, 0],
            orientation="vertical",
            brush=pg.mkBrush(44, 160, 44, 50),
            movable=False,
        )
        self.psd_plot.addItem(self.psd_embedded_region)
        self.psd_embedded_region.setVisible(False)

        # Metrics text overlay
        self.psd_metrics_text = pg.TextItem(
            color=(0, 0, 0),
            anchor=(1, 0),
            fill=pg.mkBrush(255, 255, 255, 200),
        )
        self.psd_plot.addItem(self.psd_metrics_text)

        layout = self._layout.ci.layout
        layout.setRowStretchFactor(0, 2)  # GREEN1
        layout.setRowStretchFactor(1, 2)  # GREEN2
        layout.setRowStretchFactor(2, 2)  # HR & Confidence
        layout.setRowStretchFactor(3, 1)  # Accelerometer
        layout.setColumnStretchFactor(0, 2)  # Time series plots
        layout.setColumnStretchFactor(1, 1)  # PSD plot

    def _configure_ppg_plot(self, plot: pg.PlotItem, ylabel: str) -> None:
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.setLabel("left", ylabel)
        plot.setXRange(0, WINDOW_SIZE, padding=0)
        plot.setYRange(PPG_Y_MIN, PPG_Y_MAX)
        plot.setLimits(xMin=0)
        plot.setMenuEnabled(False)
        plot.setMouseEnabled(x=False, y=False)
        plot.addLegend(offset=(10, 10))
        plot.legend.setBrush(pg.mkBrush(255, 255, 255, 180))

    def update_stats_label(self, plot: pg.PlotItem, text_item: pg.TextItem, data: np.ndarray) -> None:
        """Update text overlay with min/max/p2p/mean and anchor it top-left."""
        if data.size == 0:
            text_item.setText("")
            return

        p2p = float(np.max(data) - np.min(data))
        stats = (
            f"min={np.min(data):.0f}  max={np.max(data):.0f}  "
            f"p2p={p2p:.0f}  mean={np.mean(data):.0f}"
        )
        text_item.setText(stats)

        (xmin, xmax), (ymin, ymax) = plot.getViewBox().viewRange()
        x_pos = xmin + 0.01 * max(1.0, xmax - xmin)
        y_pos = ymax - 0.02 * max(1.0, ymax - ymin)
        text_item.setPos(x_pos, y_pos)

    def update_status_label(self, windowed: Dict[str, np.ndarray]) -> None:
        """Update the status label with latest values from all fields."""
        if not windowed or len(windowed.get("time", [])) == 0:
            return
        
        # Get the last value from each array
        green1 = windowed["GREEN1"][-1] if len(windowed["GREEN1"]) > 0 else 0
        green2 = windowed["GREEN2"][-1] if len(windowed["GREEN2"]) > 0 else 0
        hr = windowed["HR"][-1] if len(windowed["HR"]) > 0 else 0
        hr_conf = windowed["HR_Conf"][-1] if len(windowed["HR_Conf"]) > 0 else 0
        x = windowed["X"][-1] if len(windowed["X"]) > 0 else 0
        y = windowed["Y"][-1] if len(windowed["Y"]) > 0 else 0
        z = windowed["Z"][-1] if len(windowed["Z"]) > 0 else 0
        
        status_text = (
            f"<b>Latest Values:</b>  "
            f"HR: <b>{hr:.0f}</b> bpm  |  "
            f"HR_Conf: <b>{hr_conf:.0f}</b>%  |  "
            f"GREEN1: {green1:.0f}  |  "
            f"GREEN2: {green2:.0f}  |  "
            f"Accel: X={x:.0f}, Y={y:.0f}, Z={z:.0f} mg"
        )
        self.status_label.setText(status_text)

    def closeEvent(self, event) -> None:  # pragma: no cover - GUI callback
        if callable(self._on_close):
            self._on_close()
        super().closeEvent(event)

    def reset_display(self) -> None:
        """Clear plotted data and overlays."""
        for curve in (
            self.green1_curve,
            self.green2_curve,
            self.hr_curve,
            self.acc_curve_x,
            self.acc_curve_y,
            self.acc_curve_z,
        ):
            curve.setData([], [])

        self.hr_conf_scatter.setData([], [])
        self.green1_text.setText("")
        self.green2_text.setText("")
        
        # Clear PSD plot
        self.psd_curve_green1.setData([], [])
        self.psd_curve_green2.setData([], [])
        self.psd_peak_line.setVisible(False)
        self.psd_embedded_line.setVisible(False)
        self.psd_embedded_region.setVisible(False)
        self.psd_metrics_text.setHtml("")

    def use_embedded_hr_metrics(self) -> bool:
        return self.use_embedded_checkbox.isChecked()

    def _handle_use_embedded_toggled(self, checked: bool) -> None:
        mode = "embedded HR" if checked else "PSD peak"
        log.info("PSD metrics reference switched to %s", mode)

    def update_psd_plot(
        self,
        freqs_green1: np.ndarray,
        psd_green1: np.ndarray,
        freqs_green2: np.ndarray,
        psd_green2: np.ndarray,
        hr_bpm: float,
        hr_conf: float,
        metrics_green1: Dict[str, Dict[str, object]],
        metrics_green2: Dict[str, Dict[str, object]],
        use_embedded_reference: bool,
    ) -> None:
        """Update PSD plot with new data and metrics."""

        def _fmt(value: object, pattern: str, fallback: str = "N/A") -> str:
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                return fallback
            return pattern.format(numeric) if np.isfinite(numeric) else fallback

        # Update curves
        if freqs_green1.size > 0:
            self.psd_curve_green1.setData(freqs_green1, psd_green1)
        if freqs_green2.size > 0:
            self.psd_curve_green2.setData(freqs_green2, psd_green2)

        # Persist latest metrics for snapshot/export
        self._latest_psd_metrics = {
            "GREEN1": metrics_green1,
            "GREEN2": metrics_green2,
            "hr_bpm": hr_bpm,
            "hr_conf": hr_conf,
            "use_embedded_reference": use_embedded_reference,
        }

        primary_channel = self.psd_channel_combo.currentText()
        channel_metrics = metrics_green1 if primary_channel == "GREEN1" else metrics_green2
        active_metrics = channel_metrics["active"]
        peak_metrics = channel_metrics["peak"]

        # Update PSD peak marker (solid line)
        peak_freq = float(peak_metrics.get("peak_freq_hz", float("nan")))
        peak_color = self.channel_colors.get(primary_channel, "#1f77b4")
        self.psd_peak_line.setPen(pg.mkPen(color=peak_color, width=2))
        if np.isfinite(peak_freq) and PSD_FREQ_MIN <= peak_freq <= PSD_FREQ_MAX:
            self.psd_peak_line.setPos(peak_freq)
            self.psd_peak_line.setVisible(True)
        else:
            self.psd_peak_line.setVisible(False)

        # Update embedded HR marker (dashed)
        f_hr = hr_bpm / 60.0 if hr_bpm > 0 else float("nan")
        if np.isfinite(f_hr) and PSD_FREQ_MIN <= f_hr <= PSD_FREQ_MAX:
            self.psd_embedded_line.setPos(f_hr)
            self.psd_embedded_line.setVisible(True)

            freq_tol = float(peak_metrics.get("freq_tolerance_hz", PSD_PEAK_TOLERANCE_HZ))
            freq_tol = max(freq_tol, PSD_PEAK_TOLERANCE_HZ)
            self.psd_embedded_region.setRegion(
                [
                    max(PSD_FREQ_MIN, f_hr - freq_tol),
                    min(PSD_FREQ_MAX, f_hr + freq_tol),
                ]
            )
            self.psd_embedded_region.setVisible(True)
        else:
            self.psd_embedded_line.setVisible(False)
            self.psd_embedded_region.setVisible(False)

        # Compose metrics text
        snr = float(active_metrics.get("snr_db", float("nan")))
        snr_status = "N/A"
        snr_color = "gray"
        if np.isfinite(snr):
            if snr >= 10.0:
                snr_color, snr_status = "green", "GOOD"
            elif snr >= 5.0:
                snr_color, snr_status = "orange", "WARN"
            else:
                snr_color, snr_status = "red", "POOR"
        snr_center = active_metrics.get("center_label", "peak")
        snr_line = (
            f"<span style='color: {snr_color};'>SNR ({snr_center}): "
            f"{snr:.1f} dB ({snr_status})</span>"
            if np.isfinite(snr)
            else "<span style='color: gray;'>SNR: N/A (insufficient data)</span>"
        )

        peak_width = float(peak_metrics.get("peak_width_hz", float("nan")))
        peak_prom = float(peak_metrics.get("peak_prom_db", float("nan")))
        noise_floor = float(active_metrics.get("noise_floor_db", peak_metrics.get("noise_floor_db", float("nan"))))
        delta_f = float(peak_metrics.get("delta_f_hz", float("nan")))
        signal_bw = float(active_metrics.get("signal_bandwidth_hz", float("nan")))

        hardware_lines = [
            f"<b>{primary_channel} Hardware Metrics ({'embedded HR' if use_embedded_reference else 'PSD peak'})</b>",
            f"Peak HR: {_fmt(peak_metrics.get('peak_bpm', float('nan')), '{:.1f}')} bpm "
            f"({_fmt(peak_freq, '{:.2f}')} Hz)",
            f"Peak power: {_fmt(peak_metrics.get('peak_power_db', float('nan')), '{:.1f}')} dB/Hz",
            f"Peak width: {_fmt(peak_width, '{:.2f}')} Hz",
            f"Peak prominence: {_fmt(peak_prom, '{:.1f}')} dB",
            snr_line,
            f"Noise floor: {_fmt(noise_floor, '{:.1f}')} dB/Hz",
            f"Signal bandwidth: {_fmt(signal_bw, '{:.2f}')} Hz",
            f"Δf: {_fmt(delta_f, '{:.3f}')} Hz | fs: {_fmt(active_metrics.get('fs_hz', float('nan')), '{:.1f}')} Hz",
        ]

        mismatch_flag = bool(peak_metrics.get("mismatch", False))
        mismatch_reason = peak_metrics.get("mismatch_reason", "")
        mismatch_diff = float(peak_metrics.get("mismatch_bpm_diff", float("nan")))
        tolerance_bpm = float(peak_metrics.get("tolerance_bpm", float("nan")))

        mismatch_text = (
            f"<span style='color: {'red' if mismatch_flag else 'green'};'>"
            f"Mismatch: {mismatch_flag} "
            f"(Δ={_fmt(mismatch_diff, '{:.1f}')} bpm, tol={_fmt(tolerance_bpm, '{:.1f}')} bpm)"
            "</span>"
        )
        if mismatch_reason:
            mismatch_text += f"<br><span style='color: gray;'>{mismatch_reason}</span>"

        embedded_lines = [
            "<b>Embedded Algorithm:</b>",
            f"HR: {_fmt(hr_bpm, '{:.1f}')} bpm",
            f"HR_Conf: {_fmt(hr_conf, '{:.0f}')}%",
            mismatch_text,
        ]

        metrics_html = "<br>".join(hardware_lines + [""] + embedded_lines)
        self.psd_metrics_text.setHtml(metrics_html)

        # Position metrics text in top-right corner
        (xmin, xmax), (ymin, ymax) = self.psd_plot.getViewBox().viewRange()
        x_pos = xmax - 0.02 * (xmax - xmin)
        y_pos = ymax - 0.02 * (ymax - ymin)
        self.psd_metrics_text.setPos(x_pos, y_pos)

        # Auto-scale Y if not locked
        if not self.psd_lock_scale_checkbox.isChecked():
            all_psd = (
                np.concatenate([psd_green1, psd_green2])
                if psd_green1.size > 0 and psd_green2.size > 0
                else (psd_green1 if psd_green1.size > 0 else psd_green2)
            )
            if all_psd.size > 0:
                ymin_data = float(np.min(all_psd))
                ymax_data = float(np.max(all_psd))
                span = max(10.0, ymax_data - ymin_data)
                self.psd_plot.setYRange(ymin_data - 5, ymin_data + span + 5, padding=0)

    def _handle_snapshot_clicked(self) -> None:
        """Save current PSD snapshot."""
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"psd_snapshot_{timestamp}.png"
        
        try:
            if self._latest_psd_metrics is None:
                log.warning("No PSD metrics available yet; snapshot aborted.")
                return

            # Export the PSD plot
            exporter = ImageExporter(self.psd_plot.scene())
            exporter.export(filename)
            log.info("Saved PSD snapshot to %s", filename)

            metrics_file = f"psd_metrics_{timestamp}.csv"
            fieldnames = [
                "channel",
                "timestamp",
                "reference_mode",
                "peak_bpm",
                "peak_power_dB",
                "snr_dB",
                "peak_width_Hz",
                "peak_prom_dB",
                "embedded_hr_bpm",
                "embedded_hr_conf",
                "mismatch_bool",
                "mismatch_bpm_diff",
                "fs_hz",
                "nperseg",
                "delta_f_hz",
                "signal_bandwidth_hz",
            ]

            def _csv_value(value: object, precision: int = 3) -> str:
                try:
                    numeric = float(value)
                except (TypeError, ValueError):
                    return ""
                if not np.isfinite(numeric):
                    return ""
                return f"{numeric:.{precision}f}"

            with open(metrics_file, "w", newline="") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                writer.writeheader()
                for channel in ("GREEN1", "GREEN2"):
                    channel_metrics = self._latest_psd_metrics.get(channel)
                    if not channel_metrics:
                        continue
                    active = channel_metrics["active"]
                    peak = channel_metrics["peak"]
                    writer.writerow(
                        {
                            "channel": channel,
                            "timestamp": timestamp,
                            "reference_mode": active.get("center_label", "peak"),
                            "peak_bpm": _csv_value(peak.get("peak_bpm"), 2),
                            "peak_power_dB": _csv_value(peak.get("peak_power_db"), 2),
                            "snr_dB": _csv_value(active.get("snr_db"), 2),
                            "peak_width_Hz": _csv_value(peak.get("peak_width_hz"), 3),
                            "peak_prom_dB": _csv_value(peak.get("peak_prom_db"), 2),
                            "embedded_hr_bpm": _csv_value(self._latest_psd_metrics.get("hr_bpm"), 2),
                            "embedded_hr_conf": _csv_value(self._latest_psd_metrics.get("hr_conf"), 1),
                            "mismatch_bool": str(bool(peak.get("mismatch", False))),
                            "mismatch_bpm_diff": _csv_value(peak.get("mismatch_bpm_diff"), 2),
                            "fs_hz": _csv_value(peak.get("fs_hz"), 2),
                            "nperseg": _csv_value(peak.get("nperseg"), 0),
                            "delta_f_hz": _csv_value(peak.get("delta_f_hz"), 4),
                            "signal_bandwidth_hz": _csv_value(active.get("signal_bandwidth_hz"), 4),
                        }
                    )
            log.info("Saved PSD metrics to %s", metrics_file)
        except Exception as exc:
            log.error("Failed to save snapshot: %s", exc)

    def _handle_clear_clicked(self) -> None:
        if callable(self._on_clear):
            self._on_clear()

    def _handle_pause_clicked(self, checked: bool) -> None:
        if callable(self._on_pause):
            self._on_pause(checked)
        
        if checked:
            self.pause_button.setText("Resume")
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #fff3cd;
                    border: 1px solid #ffc107;
                    border-radius: 3px;
                    padding: 8px;
                    font-family: monospace;
                    font-size: 11px;
                }
            """)
        else:
            self.pause_button.setText("Pause")
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #f0f0f0;
                    border: 1px solid #ccc;
                    border-radius: 3px;
                    padding: 8px;
                    font-family: monospace;
                    font-size: 11px;
                }
            """)


class HRPlotController(QtCore.QObject):
    """Controller coordinating RTT reading and fast PyQtGraph updates."""

    def __init__(self, update_interval_ms: int = 16):
        super().__init__()
        self.window = HRPlotWindow(on_close=self.stop, on_clear=self.reset_stream, on_pause=self.toggle_pause)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(update_interval_ms)
        self.timer.timeout.connect(self.update_plots)
        self._last_autoscale = 0.0
        self._last_psd_update = 0.0
        self._paused = False

    def start(self) -> None:
        self.window.show()
        self.timer.start()

    def stop(self) -> None:
        if self.timer.isActive():
            self.timer.stop()

    def toggle_pause(self, paused: bool) -> None:
        """Pause or resume data collection and plotting."""
        self._paused = paused
        if paused:
            log.info("Data collection paused")
        else:
            log.info("Data collection resumed")

    def reset_stream(self) -> None:
        clear_stream_buffers()
        self.window.reset_display()
        self._last_autoscale = 0.0
        self._last_psd_update = 0.0

    def update_plots(self) -> None:  # pragma: no cover - requires hardware
        # Skip data reading and plotting if paused
        if self._paused:
            return
        
        read_rtt_data()
        windowed = extract_windowed_data()
        if not windowed:
            return

        time_display = windowed["time"]
        td, y1 = decimate(time_display, windowed["GREEN1"], DRAW_MAX_POINTS)
        self.window.green1_curve.setData(td, y1, connect="all")

        td, y2 = decimate(time_display, windowed["GREEN2"], DRAW_MAX_POINTS)
        self.window.green2_curve.setData(td, y2, connect="all")

        # Plot HR and HR_Conf (now with carried-forward values, no zeros to filter)
        td, hr = decimate(time_display, windowed["HR"], DRAW_MAX_POINTS)
        self.window.hr_curve.setData(td, hr, connect="finite")
        
        td_conf, hr_conf = decimate(time_display, windowed["HR_Conf"], DRAW_MAX_POINTS)
        self.window.hr_conf_scatter.setData(td_conf, hr_conf)

        td, x = decimate(time_display, windowed["X"], DRAW_MAX_POINTS)
        self.window.acc_curve_x.setData(td, x, connect="all")
        td, y = decimate(time_display, windowed["Y"], DRAW_MAX_POINTS)
        self.window.acc_curve_y.setData(td, y, connect="all")
        td, z = decimate(time_display, windowed["Z"], DRAW_MAX_POINTS)
        self.window.acc_curve_z.setData(td, z, connect="all")

        now = time.monotonic()
        if now - self._last_autoscale >= PPG_AUTOSCALE_EVERY_S:
            self._autoscale_ppg(windowed["GREEN1"], self.window.ppg1_plot)
            self._autoscale_ppg(windowed["GREEN2"], self.window.ppg2_plot)
            self._autoscale_hr(windowed["HR"], self.window.hr_plot)
            self._last_autoscale = now

        self.window.update_stats_label(self.window.ppg1_plot, self.window.green1_text, windowed["GREEN1"])
        self.window.update_stats_label(self.window.ppg2_plot, self.window.green2_text, windowed["GREEN2"])
        self.window.update_status_label(windowed)
        
        # Update PSD at slower rate (2 Hz max)
        if now - self._last_psd_update >= PSD_UPDATE_INTERVAL:
            self._update_psd(windowed)
            self._last_psd_update = now

    def _update_psd(self, windowed: Dict[str, np.ndarray]) -> None:
        """Compute and update PSD plot."""
        time_data = windowed["time"]
        
        if time_data.size < 100:  # Need sufficient data
            return
        
        # Calculate effective sample rate using median spacing for robustness
        diffs = np.diff(time_data)
        valid_diffs = diffs[diffs > 0]
        if valid_diffs.size == 0:
            return
        median_dt = float(np.median(valid_diffs))
        if not np.isfinite(median_dt) or median_dt <= 0:
            return
        sample_rate = 1.0 / median_dt
        
        # Get current HR
        hr_bpm = float(windowed["HR"][-1]) if windowed["HR"].size > 0 else 0.0
        hr_conf = float(windowed["HR_Conf"][-1]) if windowed["HR_Conf"].size > 0 else 0.0
        
        use_embedded_reference = self.window.use_embedded_hr_metrics()

        # Compute PSD for GREEN1
        freqs_green1, psd_linear_green1, psd_db_green1, nperseg_green1 = compute_psd(
            time_data,
            windowed["GREEN1"],
            sample_rate,
        )
        metrics_green1_peak = compute_psd_metrics(
            freqs_green1,
            psd_linear_green1,
            psd_db_green1,
            sample_rate,
            nperseg_green1,
            hr_bpm,
            hr_conf,
            use_embedded_reference=False,
        )
        metrics_green1_active = (
            compute_psd_metrics(
                freqs_green1,
                psd_linear_green1,
                psd_db_green1,
                sample_rate,
                nperseg_green1,
                hr_bpm,
                hr_conf,
                use_embedded_reference=True,
            )
            if use_embedded_reference
            else metrics_green1_peak
        )

        # Compute PSD for GREEN2
        freqs_green2, psd_linear_green2, psd_db_green2, nperseg_green2 = compute_psd(
            time_data,
            windowed["GREEN2"],
            sample_rate,
        )
        metrics_green2_peak = compute_psd_metrics(
            freqs_green2,
            psd_linear_green2,
            psd_db_green2,
            sample_rate,
            nperseg_green2,
            hr_bpm,
            hr_conf,
            use_embedded_reference=False,
        )
        metrics_green2_active = (
            compute_psd_metrics(
                freqs_green2,
                psd_linear_green2,
                psd_db_green2,
                sample_rate,
                nperseg_green2,
                hr_bpm,
                hr_conf,
                use_embedded_reference=True,
            )
            if use_embedded_reference
            else metrics_green2_peak
        )

        # Update the plot
        self.window.update_psd_plot(
            freqs_green1,
            psd_db_green1,
            freqs_green2,
            psd_db_green2,
            hr_bpm,
            hr_conf,
            {
                "active": metrics_green1_active,
                "peak": metrics_green1_peak,
            },
            {
                "active": metrics_green2_active,
                "peak": metrics_green2_peak,
            },
            use_embedded_reference,
        )

    def _autoscale_ppg(self, data: np.ndarray, plot: pg.PlotItem) -> None:
        if data.size < 3:
            return
        ymin = float(np.min(data))
        ymax = float(np.max(data))
        span = max(PPG_MIN_SPAN, ymax - ymin)
        pad = max(1.0, span * PPG_Y_PADDING_FRACTION)
        plot.setYRange(ymin - pad, ymin + span + pad, padding=0)

    def _autoscale_hr(self, data: np.ndarray, plot: pg.PlotItem) -> None:
        """Auto-scale the HR plot based on current data, filtering out zero values."""
        if data.size < 3:
            return
        
        # Filter out zero values for auto-scaling
        valid_data = data[data > 0]
        if valid_data.size < 3:
            return
        
        ymin = float(np.min(valid_data))
        ymax = float(np.max(valid_data))
        
        # Ensure minimum span of 20 bpm for readability
        span = max(20.0, ymax - ymin)
        pad = max(5.0, span * 0.15)  # 15% padding
        
        # Set range with padding
        plot.setYRange(ymin - pad, ymin + span + pad, padding=0)


def cleanup() -> None:
    """Cleanup resources."""
    global jlink
    if jlink and jlink.connected():
        log.info("Closing JLink connection...")
        jlink.close()
    log.info("Cleanup complete.")


def main() -> None:
    """Main entry point."""
    plotter: Optional[HRPlotController] = None
    try:
        init_jlink()

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        plotter = HRPlotController()
        plotter.start()

        log.info("Starting real-time plot. Close the window to exit.")
        app.exec()

    except KeyboardInterrupt:
        log.info("Interrupted by user.")
    except Exception as exc:  # pragma: no cover - defensive logging
        log.error("Error: %s", exc, exc_info=True)
    finally:
        if plotter:
            plotter.stop()
        cleanup()


if __name__ == "__main__":
    main()
