# PSD Panel Quick Start Guide

## What is the PSD Panel?

The Power Spectral Density (PSD) panel provides real-time frequency analysis of your PPG signals (GREEN1 and GREEN2). It helps you:
- **Identify dominant frequency peaks** (should match heart rate)
- **Assess signal quality** via SNR and noise floor metrics
- **Detect issues** like broad peaks or frequency mismatches

## UI Layout

```
┌─────────────────────────────────┬──────────────────┐
│  GREEN1 Time Series             │                  │
├─────────────────────────────────┤                  │
│  GREEN2 Time Series             │   PSD 0–5 Hz     │
├─────────────────────────────────┤   (Frequency     │
│  Heart Rate & Confidence        │    Analysis)     │
├─────────────────────────────────┤                  │
│  Accelerometer                  │                  │
└─────────────────────────────────┴──────────────────┘
```

## Key Elements

### 1. PSD Traces
- **Blue line**: GREEN1 channel frequency spectrum
- **Red line**: GREEN2 channel frequency spectrum
- **X-axis**: Frequency in Hz (0-5 Hz range covers 0-300 bpm)
- **Y-axis**: Power in dB/Hz

### 2. PSD Peak Marker (Solid Line)
- Tracks the dominant frequency found in the PSD (hardware-derived HR)
- Color matches the currently selected channel (BLUE for GREEN1, RED for GREEN2)

### 3. Embedded HR Overlay (Dashed Line)
- Shows the heart rate reported by the firmware algorithm
- Shaded region indicates the mismatch tolerance band
- Toggle whether calculations reference the embedded HR or the PSD peak (see Controls)

### 4. Metrics Box (Top-Right)
Two-group summary for the selected channel:
- **Hardware metrics (always in use)**
  - Peak HR / frequency, peak power, FWHM width, prominence
  - SNR calculated around the PSD peak (unless the toggle forces embedded HR)
  - Noise floor, signal bandwidth, `Δf`, and the effective sample rate derived from device timestamps
- **Embedded algorithm diagnostics**
  - Embedded HR and confidence %
  - Mismatch flag with numeric difference vs. PSD peak and tolerance
- Status colors:
  - 🟢 **SNR ≥ 10 dB** good, 🟠 **5-9 dB** marginal, 🔴 **<5 dB** poor
  - Mismatch text shows red when |Δ| exceeds the adaptive tolerance

## Controls

### Lock Y-scale (Checkbox)
- **Checked (default)**: Y-axis stays at -60 to 40 dB
- **Unchecked**: Y-axis auto-scales to fit data

### Channel Dropdown (GREEN1/GREEN2)
- Selects which channel's metrics are displayed in the metrics box
- Both channels always plotted, this only affects which metrics appear

### Use Embedded HR for Metrics (Checkbox)
- **Off (default)**: Hardware metrics center on the PSD peak
- **On**: Hardware metrics (SNR/noise bandwidth) are recomputed using the embedded HR frequency. Useful for firmware validation.

### Snapshot Button
- Saves current PSD plot as PNG image
- Saves hardware + embedded metrics (for both channels) to CSV
- Files named: `psd_snapshot_YYYYMMDD_HHMMSS.png` and `psd_metrics_YYYYMMDD_HHMMSS.csv`

## Interpreting the PSD

### Good Signal
- Sharp, tall peak at HR frequency
- SNR ≥ 10 dB (green)
- Peak width < 0.4 Hz
- Peak frequency matches HR marker (no mismatch badge)

### Poor Signal Examples
1. **Noisy Signal**
   - Broad, low peaks
   - SNR < 5 dB (red)
   - "BROAD PEAK" warning
   
2. **Motion Artifact**
   - Multiple peaks or energy at harmonics/subharmonics
   - PSD peak vs embedded HR mismatch flagged (red)
   - Consider cross-checking accelerometer trace

3. **Low Perfusion**
   - Very low overall power
   - High noise floor
   - Barely visible peaks

## Tips for Best Results

1. **Wait for stable data**: PSD needs ~20 seconds of data for reliable analysis
2. **Compare channels**: GREEN1 vs GREEN2 can show which sensor has better contact
3. **Monitor SNR**: If SNR drops below 10 dB, check sensor placement
4. **Check peak width**: Narrow peaks (<0.2 Hz) indicate stable heart rate
5. **Use snapshots**: Capture good/bad examples for comparison

## Technical Details

- **Sample timing**: Uses device-provided timestamps (`TS,<ms_since_boot>`) for accurate sampling frequency, with automatic monotonic fallback if duplicates occur
- **Update rate**: PSD refreshes every 0.5 seconds (2 Hz)
- **Window**: ~8-second Hann windows with ≥4·fs segment length, 50% overlap (Welch)
- **High-pass**: Light 0.2 Hz filter applied to PSD path only (time plots remain raw)
- **Valid analysis band**: 0.3–5.0 Hz (≈18–300 bpm)

## Troubleshooting

**Q: PSD shows flat line at bottom**
- Not enough data collected yet (wait ~20 seconds)
- Signal is DC-only (check sensor connection)

**Q: Many peaks but none match HR**
- Motion artifact or poor contact
- Try adjusting sensor position

**Q: Different peaks for GREEN1 vs GREEN2**
- One sensor may have better contact
- Use channel dropdown to compare metrics

**Q: SNR always shows N/A**
- No usable HR peak in the PSD, or the toggle points at an embedded HR outside the analysis band
- Check the PSD plot to confirm where the dominant energy is
