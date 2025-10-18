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


class HRPlotWindow(QtWidgets.QMainWindow):
    """PyQtGraph window hosting the live plots."""

    def __init__(self, on_close=None, on_clear=None, on_pause=None):
        super().__init__()
        self._on_close = on_close
        self._on_clear = on_clear
        self._on_pause = on_pause

        self.setWindowTitle("Heart Rate Sensor - Real-time Raw Data (PyQtGraph)")
        self.resize(1280, 900)

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

        layout = self._layout.ci.layout
        layout.setRowStretchFactor(0, 2)  # GREEN1
        layout.setRowStretchFactor(1, 2)  # GREEN2
        layout.setRowStretchFactor(2, 2)  # HR & Confidence (increased from 1 to 2)
        layout.setRowStretchFactor(3, 1)  # Accelerometer

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
