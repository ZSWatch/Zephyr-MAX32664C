#!/usr/bin/env python3
"""
Test script for PSD computation functions.
"""

import numpy as np
from scipy import signal as scipy_signal

# Simulate the functions from raw_hr_data_plotter.py
PSD_WINDOW_LENGTH = 8.0
PSD_OVERLAP = 0.5
PSD_HIGHPASS_CUTOFF_HZ = 0.2

def compute_psd(time_data, signal_data, sample_rate):
    """Compute Power Spectral Density using Welch's method."""
    if signal_data.size < 100 or sample_rate <= 0:
        return np.array([]), np.array([]), np.array([]), 0

    signal_detrended = signal_data - np.mean(signal_data)
    nyquist = 0.5 * sample_rate
    cutoff = min(PSD_HIGHPASS_CUTOFF_HZ, nyquist * 0.99)
    if 0 < cutoff < nyquist:
        sos = scipy_signal.butter(2, cutoff / nyquist, btype="highpass", output="sos")
        padlen = min(250, signal_detrended.size - 1)
        if padlen >= 1:
            signal_detrended = scipy_signal.sosfiltfilt(sos, signal_detrended, padlen=padlen)
        else:
            signal_detrended = scipy_signal.sosfilt(sos, signal_detrended)

    target_nperseg = int(PSD_WINDOW_LENGTH * sample_rate)
    min_required = int(4 * sample_rate)
    nperseg = min(signal_detrended.size, max(64, target_nperseg, min_required))
    if nperseg <= 1:
        return np.array([]), np.array([]), np.array([]), 0

    try:
        freqs, psd = scipy_signal.welch(
            signal_detrended,
            fs=sample_rate,
            window="hann",
            nperseg=nperseg,
            noverlap=int(nperseg * PSD_OVERLAP),
            scaling="density",
        )
        psd_db = 10 * np.log10(psd + 1e-12)
        return freqs, psd, psd_db, nperseg
    except Exception as exc:
        print(f"PSD computation failed: {exc}")
        return np.array([]), np.array([]), np.array([]), 0

def test_psd_with_synthetic_signal():
    """Test PSD computation with a synthetic heart rate signal."""
    # Create synthetic PPG signal: 1 Hz (60 bpm) with noise
    sample_rate = 100  # Hz
    duration = 20  # seconds
    t = np.linspace(0, duration, int(sample_rate * duration))
    
    # Heart rate component at 1 Hz (60 bpm)
    hr_freq = 1.0
    ppg_signal = 100000 + 50000 * np.sin(2 * np.pi * hr_freq * t)
    
    # Add some noise
    noise = np.random.normal(0, 5000, ppg_signal.shape)
    ppg_signal += noise
    
    # Compute PSD
    freqs, _, psd_db, _ = compute_psd(t, ppg_signal, sample_rate)
    
    print(f"Computed PSD with {len(freqs)} frequency bins")
    print(f"Frequency range: {freqs[0]:.3f} - {freqs[-1]:.3f} Hz")
    
    # Find peak in physiological range (0.5-3 Hz for 30-180 bpm)
    mask = (freqs >= 0.5) & (freqs <= 3.0)
    if np.any(mask):
        freqs_roi = freqs[mask]
        psd_roi = psd_db[mask]
        peak_idx = np.argmax(psd_roi)
        peak_freq = freqs_roi[peak_idx]
        peak_bpm = peak_freq * 60
        
        print(f"Peak frequency: {peak_freq:.3f} Hz ({peak_bpm:.1f} bpm)")
        print(f"Expected: {hr_freq:.3f} Hz (60 bpm)")
        print(f"Error: {abs(peak_freq - hr_freq):.3f} Hz")
        
        if abs(peak_freq - hr_freq) < 0.1:
            print("✓ PSD computation working correctly!")
        else:
            print("✗ PSD computation may have issues")
    else:
        print("✗ No valid frequency range found")

if __name__ == "__main__":
    test_psd_with_synthetic_signal()
