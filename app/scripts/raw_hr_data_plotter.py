#!/usr/bin/env python3
"""
Real-time plotter for heart rate sensor raw data (PPG and accelerometer).
Streams data via RTT and renders with PyQtGraph for high-frequency updates.
"""

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
PSD_HR_TOLERANCE = 0.1  # Hz around HR frequency for SNR calculation
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
    global line_buffer, sample_count, write_idx

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

            elapsed_time = (current_milli_time() - start_time) / 1000.0
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
    global line_buffer, sample_count, write_idx, start_time
    for array in buffers.values():
        array.fill(0)
    line_buffer = ""
    sample_count = 0
    write_idx = 0
    start_time = current_milli_time()
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


def compute_psd(time_data: np.ndarray, signal_data: np.ndarray, 
                sample_rate: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute Power Spectral Density using Welch's method.
    
    Args:
        time_data: Time array (not used, but kept for API consistency)
        signal_data: Signal values
        sample_rate: Effective sample rate in Hz
    
    Returns:
        (frequencies, psd_db): frequency array and PSD in dB/Hz
    """
    if signal_data.size < 100:  # Need minimum samples
        return np.array([]), np.array([])
    
    # Detrend: remove mean
    signal_detrended = signal_data - np.mean(signal_data)
    
    # Calculate nperseg for ~8 second windows
    nperseg = min(int(PSD_WINDOW_LENGTH * sample_rate), signal_detrended.size)
    if nperseg < 64:  # Minimum segment length
        nperseg = min(64, signal_detrended.size)
    
    # Compute PSD using Welch's method with Hann window
    try:
        freqs, psd = signal.welch(
            signal_detrended,
            fs=sample_rate,
            window='hann',
            nperseg=nperseg,
            noverlap=int(nperseg * PSD_OVERLAP),
            scaling='density',
        )
        
        # Convert to dB/Hz, avoiding log(0)
        psd_db = 10 * np.log10(psd + 1e-12)
        
        return freqs, psd_db
    except Exception as exc:
        log.warning("PSD computation failed: %s", exc)
        return np.array([]), np.array([])


def compute_psd_metrics(freqs: np.ndarray, psd_db: np.ndarray, 
                        hr_bpm: float) -> Dict[str, float]:
    """
    Compute PSD quality metrics.
    
    Args:
        freqs: Frequency array in Hz
        psd_db: PSD in dB/Hz
        hr_bpm: Current heart rate in bpm
    
    Returns:
        Dictionary with metrics: peak_freq_hz, peak_freq_bpm, snr_db, 
        noise_floor_db, peak_width_hz, hr_mismatch
    """
    metrics = {
        "peak_freq_hz": 0.0,
        "peak_freq_bpm": 0.0,
        "snr_db": 0.0,
        "noise_floor_db": 0.0,
        "peak_width_hz": 0.0,
        "hr_mismatch": False,
    }
    
    if freqs.size == 0 or psd_db.size == 0:
        return metrics
    
    # Limit analysis to 0.3-5.0 Hz (18-300 bpm)
    valid_mask = (freqs >= 0.3) & (freqs <= 5.0)
    if not np.any(valid_mask):
        return metrics
    
    freqs_valid = freqs[valid_mask]
    psd_valid = psd_db[valid_mask]
    
    # Find peak frequency
    peak_idx = np.argmax(psd_valid)
    peak_freq = freqs_valid[peak_idx]
    metrics["peak_freq_hz"] = float(peak_freq)
    metrics["peak_freq_bpm"] = float(peak_freq * 60.0)
    
    # Compute HR frequency
    f_hr = hr_bpm / 60.0 if hr_bpm > 0 else 0.0
    
    # SNR computation if HR is valid
    if 0.3 <= f_hr <= 5.0:
        # Signal power: within ±0.1 Hz around HR frequency
        signal_mask = np.abs(freqs_valid - f_hr) <= PSD_HR_TOLERANCE
        if np.any(signal_mask):
            # Use linear power for SNR calculation
            psd_linear = 10 ** (psd_valid / 10.0)
            signal_power = np.mean(psd_linear[signal_mask])
            
            # Noise power: rest of the band excluding HR region
            noise_mask = ~signal_mask
            if np.any(noise_mask):
                noise_power = np.mean(psd_linear[noise_mask])
                if noise_power > 0:
                    snr_linear = signal_power / noise_power
                    metrics["snr_db"] = float(10 * np.log10(snr_linear))
                    metrics["noise_floor_db"] = float(10 * np.log10(noise_power))
        
        # Check for mismatch
        if abs(peak_freq - f_hr) > 0.1:
            metrics["hr_mismatch"] = True
    else:
        # If no valid HR, just compute noise floor
        psd_linear = 10 ** (psd_valid / 10.0)
        metrics["noise_floor_db"] = float(10 * np.log10(np.median(psd_linear)))
    
    # Compute FWHM (Full Width at Half Maximum) for peak width
    try:
        peak_power = psd_valid[peak_idx]
        half_max = peak_power - 3.0  # -3 dB from peak (half power)
        
        # Find points above half maximum
        above_half = psd_valid >= half_max
        if np.sum(above_half) >= 2:
            # Find the span of frequencies above half max around the peak
            indices = np.where(above_half)[0]
            # Get continuous region containing peak
            peak_region = []
            for i in indices:
                if abs(i - peak_idx) <= len(indices):
                    peak_region.append(i)
            
            if len(peak_region) >= 2:
                width_hz = freqs_valid[max(peak_region)] - freqs_valid[min(peak_region)]
                metrics["peak_width_hz"] = float(width_hz)
    except Exception:
        pass  # Keep default 0.0
    
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
        
        self.snapshot_button = QtWidgets.QPushButton("Snapshot")
        self.snapshot_button.clicked.connect(self._handle_snapshot_clicked)
        controls_layout.addWidget(self.snapshot_button)
        
        controls_layout.addStretch()

        central_layout.addLayout(controls_layout)

        self._layout = pg.GraphicsLayoutWidget()
        central_layout.addWidget(self._layout)
        self.setCentralWidget(central)

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
        
        # HR frequency marker (vertical line)
        self.psd_hr_line = pg.InfiniteLine(
            angle=90,
            movable=False,
            pen=pg.mkPen(color="#2ca02c", width=2, style=QtCore.Qt.PenStyle.DashLine),
            label="HR",
        )
        self.psd_plot.addItem(self.psd_hr_line)
        self.psd_hr_line.setVisible(False)
        
        # HR tolerance region (shaded)
        self.psd_hr_region = pg.LinearRegionItem(
            values=[0, 0],
            orientation='vertical',
            brush=pg.mkBrush(44, 160, 44, 50),
            movable=False,
        )
        self.psd_plot.addItem(self.psd_hr_region)
        self.psd_hr_region.setVisible(False)
        
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
        self.psd_hr_line.setVisible(False)
        self.psd_hr_region.setVisible(False)
        self.psd_metrics_text.setHtml("")

    def update_psd_plot(
        self,
        freqs_green1: np.ndarray,
        psd_green1: np.ndarray,
        freqs_green2: np.ndarray,
        psd_green2: np.ndarray,
        hr_bpm: float,
        metrics_green1: Dict[str, float],
        metrics_green2: Dict[str, float],
    ) -> None:
        """Update PSD plot with new data and metrics."""
        # Update curves
        if freqs_green1.size > 0:
            self.psd_curve_green1.setData(freqs_green1, psd_green1)
        if freqs_green2.size > 0:
            self.psd_curve_green2.setData(freqs_green2, psd_green2)
        
        # Update HR marker
        f_hr = hr_bpm / 60.0 if hr_bpm > 0 else 0.0
        if 0.3 <= f_hr <= 5.0:
            self.psd_hr_line.setPos(f_hr)
            self.psd_hr_line.setVisible(True)
            
            # Update tolerance region
            self.psd_hr_region.setRegion([
                f_hr - PSD_HR_TOLERANCE,
                f_hr + PSD_HR_TOLERANCE
            ])
            self.psd_hr_region.setVisible(True)
        else:
            self.psd_hr_line.setVisible(False)
            self.psd_hr_region.setVisible(False)
        
        # Update metrics text
        primary_channel = self.psd_channel_combo.currentText()
        metrics = metrics_green1 if primary_channel == "GREEN1" else metrics_green2
        
        # Format metrics with color coding
        snr = metrics["snr_db"]
        if snr >= 10.0:
            snr_color = "green"
            snr_status = "GOOD"
        elif snr >= 5.0:
            snr_color = "orange"
            snr_status = "WARN"
        else:
            snr_color = "red"
            snr_status = "POOR"
        
        metrics_lines = [
            f"<b>{primary_channel} Metrics:</b>",
            f"<span style='color: {snr_color};'>SNR@HR: {snr:.1f} dB ({snr_status})</span>",
            f"Peak: {metrics['peak_freq_hz']:.2f} Hz ({metrics['peak_freq_bpm']:.0f} bpm)",
            f"Noise floor: {metrics['noise_floor_db']:.1f} dB",
            f"Peak width: {metrics['peak_width_hz']:.2f} Hz",
        ]
        
        if metrics["hr_mismatch"]:
            metrics_lines.append("<span style='color: red;'><b>⚠ MISMATCH</b></span>")
        
        if metrics["peak_width_hz"] > 0.4:
            metrics_lines.append("<span style='color: orange;'><b>⚠ BROAD PEAK</b></span>")
        
        metrics_html = "<br>".join(metrics_lines)
        self.psd_metrics_text.setHtml(metrics_html)
        
        # Position metrics text in top-right corner
        (xmin, xmax), (ymin, ymax) = self.psd_plot.getViewBox().viewRange()
        x_pos = xmax - 0.02 * (xmax - xmin)
        y_pos = ymax - 0.02 * (ymax - ymin)
        self.psd_metrics_text.setPos(x_pos, y_pos)
        
        # Auto-scale Y if not locked
        if not self.psd_lock_scale_checkbox.isChecked():
            all_psd = np.concatenate([psd_green1, psd_green2]) if psd_green1.size > 0 and psd_green2.size > 0 else (psd_green1 if psd_green1.size > 0 else psd_green2)
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
            # Export the PSD plot
            exporter = pg.exporters.ImageExporter(self.psd_plot.scene())
            exporter.export(filename)
            log.info("Saved PSD snapshot to %s", filename)
            
            # Also log metrics to a text file
            metrics_file = f"psd_metrics_{timestamp}.txt"
            with open(metrics_file, "w") as f:
                f.write(f"Timestamp: {timestamp}\n")
                f.write(self.psd_metrics_text.toPlainText())
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
        
        # Calculate effective sample rate
        if time_data[-1] > time_data[0]:
            sample_rate = time_data.size / (time_data[-1] - time_data[0])
        else:
            return
        
        # Get current HR
        hr_bpm = float(windowed["HR"][-1]) if windowed["HR"].size > 0 else 0.0
        
        # Compute PSD for GREEN1
        freqs_green1, psd_green1 = compute_psd(time_data, windowed["GREEN1"], sample_rate)
        metrics_green1 = compute_psd_metrics(freqs_green1, psd_green1, hr_bpm)
        
        # Compute PSD for GREEN2
        freqs_green2, psd_green2 = compute_psd(time_data, windowed["GREEN2"], sample_rate)
        metrics_green2 = compute_psd_metrics(freqs_green2, psd_green2, hr_bpm)
        
        # Update the plot
        self.window.update_psd_plot(
            freqs_green1,
            psd_green1,
            freqs_green2,
            psd_green2,
            hr_bpm,
            metrics_green1,
            metrics_green2,
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
