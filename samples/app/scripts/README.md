# Heart Rate Plotter

## Required Python Packages

Install the dependencies into your environment (virtualenv recommended):

```bash
pip install numpy pyqtgraph pylink-square PyQt5 pyserial scipy
```

## Usage

### Real-time RTT Plotter

The `raw_hr_data_plotter.py` script connects to a Segger J-Link RTT stream and renders the live PPG and accelerometer data with PyQtGraph:

```bash
python3 raw_hr_data_plotter.py
```

For nRF5340 targets (e.g. ZSWatch), set the target device:

```bash
JLINK_TARGET_DEVICE=NRF5340_XXAA python3 raw_hr_data_plotter.py
```

Ensure a J-Link probe is attached and, if multiple probes are available, set `JLINK_SERIAL` in the environment.

### Real-time UART Plotter

The same script also supports reading data over UART:

```bash
python3 raw_hr_data_plotter.py --uart /dev/ttyUSB0
python3 raw_hr_data_plotter.py --uart /dev/ttyUSB0 --baud 115200
```

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
