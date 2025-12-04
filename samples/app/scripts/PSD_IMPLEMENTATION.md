# PSD Panel Implementation Summary

## Overview
Added a real-time Power Spectral Density (PSD) analysis panel to the heart rate plotter, providing frequency-domain visualization and quality metrics for GREEN1 and GREEN2 PPG channels.

## Key Features Implemented

### 1. PSD Computation (Welch's Method)
- **Function**: `compute_psd()`
- **Detrending**: Removes mean, applies 0.2 Hz high-pass (signal unchanged in time plots)
- **Window**: Hann window, ~8 second segments with minimum length ≥ 4·fs
- **Overlap**: 50% between segments
- **Output**: Returns frequencies, linear PSD, PSD in dB/Hz, and the segment length (`nperseg`)

### 2. Quality Metrics Computation
- **Function**: `compute_psd_metrics()`
- **Metrics calculated**:
  - **Peak frequency / BPM** from the PSD dominant bin
  - **Signal power / SNR** integrated around either the PSD peak or (optional) embedded HR
  - **Noise floor**: Median density outside the peak/harmonic bands
  - **Peak width (FWHM)** and prominence in dB
  - **Mismatch diagnostics**: |Δ| vs embedded HR with adaptive tolerance based on `Δf`
  - **Supporting metadata**: `fs`, `Δf`, signal bandwidth, tolerance in bpm, etc.

### 3. UI Components

#### PSD Plot Panel (Right Column)
- **Title**: "PSD 0–5 Hz"
- **X-axis**: Frequency 0-5 Hz
- **Y-axis**: Power in dB/Hz (default range: -60 to 40 dB)
- **Two traces**: GREEN1 (blue) and GREEN2 (red), matching time-series colors
- **PSD peak marker**: Solid line at dominant frequency (colored per channel)
- **Embedded HR marker**: Green dashed line with shaded tolerance region
- **Metrics display**: Top-right text box summarizing hardware vs embedded metrics

#### Control Buttons
- **Lock Y-scale checkbox**: Toggle between fixed and auto-scaling Y-axis (default: locked)
- **Channel dropdown**: Select primary channel for metrics display (GREEN1 or GREEN2)
- **Use embedded HR for metrics**: Optional toggle to center SNR/noise on firmware HR
- **Snapshot button**: Save current PSD plot and metrics to files

### 4. Visual Quality Indicators

#### SNR Color Coding
- **Green (GOOD)**: SNR ≥ 10 dB
- **Orange (WARN)**: SNR 5-9 dB
- **Red (POOR)**: SNR < 5 dB

#### Warning Badges
- **⚠ MISMATCH**: |PSD peak − embedded HR| exceeds adaptive tolerance
- **⚠ BROAD PEAK**: Peak width > 0.4 Hz (indicates instability)

### 5. Performance Optimizations
- **PSD refresh rate**: Maximum 2 Hz (0.5s interval) to avoid overwhelming UI
- **Independent UI refresh**: Time-series plots update at 60 FPS independent of PSD
- **Reusable plot items**: Curves and markers are reused, not recreated on each update
- **Efficient computation**: Uses scipy.signal.welch for optimized spectral analysis

### 6. Snapshot Feature
When "Snapshot" button is clicked:
- Saves PSD plot as PNG: `psd_snapshot_YYYYMMDD_HHMMSS.png`
- Saves metrics as CSV: `psd_metrics_YYYYMMDD_HHMMSS.csv`
- Includes hardware metrics for both channels plus embedded HR/Conf, fs, nperseg, Δf, mismatch flags

## Layout Changes
- Window width increased: 1280 → 1600 pixels to accommodate PSD panel
- Time-series plots occupy left column (2/3 width)
- PSD plot occupies right column (1/3 width), spanning all rows
- All time-series plots remain fully functional

## Configuration Constants Added
```python
PSD_WINDOW_LENGTH = 8.0       # Welch segment length in seconds
PSD_OVERLAP = 0.5             # 50% overlap between segments
PSD_UPDATE_INTERVAL = 0.5     # Update every 0.5s (2 Hz)
PSD_FREQ_MIN = 0.0            # Hz
PSD_FREQ_MAX = 5.0            # Hz
PSD_ANALYSIS_MIN_HZ = 0.3     # Hz lower bound for analysis band
PSD_ANALYSIS_MAX_HZ = 5.0     # Hz upper bound
PSD_PEAK_TOLERANCE_HZ = 0.12  # Base half-bandwidth around peak/embedded HR
PSD_Y_MIN = -60               # dB/Hz
PSD_Y_MAX = 40                # dB/Hz
PSD_HIGHPASS_CUTOFF_HZ = 0.2  # High-pass applied only for PSD computation
EMBEDDED_HR_CONF_THRESHOLD = 70.0  # % confidence threshold for mismatch reporting
```

## Dependencies Added
- **scipy**: For `signal.welch()` function

## Files Modified
- `raw_hr_data_plotter.py`: Main implementation (all changes)

## Files Created
- `test_psd.py`: Unit test for PSD computation (verified working)

## Usage
The PSD panel automatically updates during data collection:
1. Real-time frequency analysis shows dominant peaks
2. PSD peak marker indicates hardware-derived HR
3. Embedded HR overlay provides algorithm comparison
4. Metrics update to show hardware vs embedded performance
5. Use "Lock Y-scale" to fix axis range or allow auto-scaling
6. Select channel (GREEN1/GREEN2) or toggle embedded weighting
7. Click "Snapshot" to save PNG + CSV artifacts

## Validation
- Synthetic signal test (60 bpm sine wave) correctly identifies 1.0 Hz peak
- SNR and mismatch logic verified with injected noise and frequency offsets
- Peak detection accurate within ±0.05 Hz of expected frequency when device timestamps are present

## Integration Notes
- All existing functionality preserved
- Pause/Resume button affects PSD updates too
- Clear Data button resets PSD plot
- Device-provided timestamps (`TS,<ms_since_boot>`) are preferred; when absent, host timestamps are used with monotonic fallback
