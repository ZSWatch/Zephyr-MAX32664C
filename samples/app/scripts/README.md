# Heart Rate Plotter

## Required Python Packages

Install the dependencies into your environment (virtualenv recommended):

```bash
pip install numpy pyqtgraph pylink-square PyQt5
```

## Usage

### Real-time RTT Plotter

The `raw_hr_data_plotter.py` script connects to the Segger J-Link RTT stream and renders the live PPG and accelerometer data with PyQtGraph:

```bash
python3 raw_hr_data_plotter.py
```

Ensure a J-Link probe is attached and, if multiple probes are available, set `SERIAL_NUMBER` in the script.

### Offline Log Plotter

Collect data with the nRF Connect app:
- Bond
- Auto Connect
- Enable notifications on Nordic NUS Service
- Open logs and choose Log Level "APP"
- Observe HR data in the log
- Save the log file and copy it to your computer

Plot a saved log with the legacy matplotlib script:

```bash
pip install matplotlib numpy
python3 hr_plotter.py <log_file_path>
```

Example:
```bash
python3 hr_plotter.py "Log 2025-10-16 19_23_58.txt"
```
