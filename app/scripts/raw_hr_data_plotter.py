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
    GREEN1,33111,;GREEN2,26648,;IR1,50054,;IR2,139887,;RED1,24397,;RED2,128212,;X,-15,;Y,12,;Z,1001;...

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
            except ValueError:
                # Skip non-integer values (e.g. "bpm,72")
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

    current_time = time_view[-1]
    if current_time < WINDOW_SIZE:
        time_display = time_view
        ppg1_display = ppg1_view
        ppg2_display = ppg2_view
        accx_display = accx_view
        accy_display = accy_view
        accz_display = accz_view
    else:
        window_start = current_time - WINDOW_SIZE
        mask = time_view >= window_start
        time_display = time_view[mask] - window_start
        ppg1_display = ppg1_view[mask]
        ppg2_display = ppg2_view[mask]
        accx_display = accx_view[mask]
        accy_display = accy_view[mask]
        accz_display = accz_view[mask]

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
    }


def decimate(x: np.ndarray, y: np.ndarray, nmax: int) -> tuple[np.ndarray, np.ndarray]:
    """Thin data for plotting without heavy resampling."""
    if x.size <= nmax:
        return x, y
    step = max(1, x.size // nmax)
    return x[::step], y[::step]


class HRPlotWindow(QtWidgets.QMainWindow):
    """PyQtGraph window hosting the live plots."""

    def __init__(self, on_close=None, on_clear=None):
        super().__init__()
        self._on_close = on_close
        self._on_clear = on_clear

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

        controls_layout = QtWidgets.QHBoxLayout()
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setSpacing(6)

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
            title="GREEN1 (IR/Green depending on setup)",
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
            title="GREEN2 (paired with GREEN1)",
        )
        self._configure_ppg_plot(self.ppg2_plot, "Amplitude")
        self.green2_curve = self.ppg2_plot.plot(pen=pg.mkPen(color="#d62728", width=1), name="GREEN2")
        self.green2_curve.setClipToView(True)
        self.green2_curve.setDownsampling(auto=True, method="peak")
        self.green2_text = pg.TextItem(color=(0, 0, 0), anchor=(0, 1))
        self.ppg2_plot.addItem(self.green2_text)

        # Accelerometer plot
        self.acc_plot = self._layout.addPlot(
            row=2,
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
        layout.setRowStretchFactor(0, 2)
        layout.setRowStretchFactor(1, 2)
        layout.setRowStretchFactor(2, 1)

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

    def closeEvent(self, event) -> None:  # pragma: no cover - GUI callback
        if callable(self._on_close):
            self._on_close()
        super().closeEvent(event)

    def reset_display(self) -> None:
        """Clear plotted data and overlays."""
        for curve in (
            self.green1_curve,
            self.green2_curve,
            self.acc_curve_x,
            self.acc_curve_y,
            self.acc_curve_z,
        ):
            curve.setData([], [])

        self.green1_text.setText("")
        self.green2_text.setText("")

    def _handle_clear_clicked(self) -> None:
        if callable(self._on_clear):
            self._on_clear()


class HRPlotController(QtCore.QObject):
    """Controller coordinating RTT reading and fast PyQtGraph updates."""

    def __init__(self, update_interval_ms: int = 16):
        super().__init__()
        self.window = HRPlotWindow(on_close=self.stop, on_clear=self.reset_stream)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(update_interval_ms)
        self.timer.timeout.connect(self.update_plots)
        self._last_autoscale = 0.0

    def start(self) -> None:
        self.window.show()
        self.timer.start()

    def stop(self) -> None:
        if self.timer.isActive():
            self.timer.stop()

    def reset_stream(self) -> None:
        clear_stream_buffers()
        self.window.reset_display()
        self._last_autoscale = 0.0

    def update_plots(self) -> None:  # pragma: no cover - requires hardware
        read_rtt_data()
        windowed = extract_windowed_data()
        if not windowed:
            return

        time_display = windowed["time"]
        td, y1 = decimate(time_display, windowed["GREEN1"], DRAW_MAX_POINTS)
        self.window.green1_curve.setData(td, y1, connect="all")

        td, y2 = decimate(time_display, windowed["GREEN2"], DRAW_MAX_POINTS)
        self.window.green2_curve.setData(td, y2, connect="all")

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
            self._last_autoscale = now

        self.window.update_stats_label(self.window.ppg1_plot, self.window.green1_text, windowed["GREEN1"])
        self.window.update_stats_label(self.window.ppg2_plot, self.window.green2_text, windowed["GREEN2"])

    def _autoscale_ppg(self, data: np.ndarray, plot: pg.PlotItem) -> None:
        if data.size < 3:
            return
        ymin = float(np.min(data))
        ymax = float(np.max(data))
        span = max(PPG_MIN_SPAN, ymax - ymin)
        pad = max(1.0, span * PPG_Y_PADDING_FRACTION)
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
