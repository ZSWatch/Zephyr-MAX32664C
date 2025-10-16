# Heart Rate Plotter

## Collect data
nRF Connect App:
- Bond
- Auto Connect
- Enable notifications on Nordic NUS Service
- Open logs and choose Log Level "APP"
- See HR data coming in
- After a while save the log to file and send it you your computer.

## Usage

```bash
pip install matplotlib numpy
python3 hr_plotter.py <log_file_path>
```

Example:
```bash
python3 hr_plotter.py "Log 2025-10-16 19_23_58.txt"
```
