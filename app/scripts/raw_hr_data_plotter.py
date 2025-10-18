#!/usr/bin/env python3
"""
Real-time plotter for heart rate sensor raw data (PPG and accelerometer).
Streams data via RTT and displays it in subplots for analysis.
Uses optimized matplotlib rendering with blitting for maximum performance.
"""

import logging
import time
import numpy as np
import matplotlib as mpl

# Performance tunings BEFORE importing pyplot
mpl.rcParams.update({
    "path.simplify": True,
    "path.simplify_threshold": 0.5,
    "agg.path.chunksize": 10000,
    "figure.dpi": 100,
    "lines.antialiased": False,
})

import matplotlib.pyplot as plt
import pylink
from utils import current_milli_time

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
log = logging.getLogger(__name__)

# Configuration
TARGET_DEVICE = "NRF54L15_M33"
SERIAL_NUMBER = None
# Buffer size (fixed for numpy arrays). Should be >= WINDOW_SIZE * effective_sample_rate.
# If your sample rate is ~1kHz and WINDOW_SIZE is 10s, set this to >= 10000.
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
ACC_Y_MIN = -2000
ACC_Y_MAX = 2000

# Preallocated NumPy buffers for speed - no Python lists/deques in hot path
buffers = {
    'GREEN1': np.zeros(MAX_POINTS, dtype=np.float32),
    'IR1': np.zeros(MAX_POINTS, dtype=np.float32),
    'RED1': np.zeros(MAX_POINTS, dtype=np.float32),
    'GREEN2': np.zeros(MAX_POINTS, dtype=np.float32),
    'IR2': np.zeros(MAX_POINTS, dtype=np.float32),
    'RED2': np.zeros(MAX_POINTS, dtype=np.float32),
    'X': np.zeros(MAX_POINTS, dtype=np.float32),
    'Y': np.zeros(MAX_POINTS, dtype=np.float32),
    'Z': np.zeros(MAX_POINTS, dtype=np.float32),
    'time': np.zeros(MAX_POINTS, dtype=np.float32),
}

# Global variables
jlink = None
start_time = None
line_buffer = ""
sample_count = 0
write_idx = 0  # Circular buffer index
last_autoscale_time = 0.0


def parse_data_line(line):
    """
    Parse a line of data in the format:
    GREEN1,33111,;GREEN2,26648,;IR1,50054,;IR2,139887,;RED1,24397,;RED2,128212,;X,-15,;Y,12,;Z,1001;...
    
    Returns a dictionary with parsed values or None if parsing fails.
    """
    data = {}
    try:
        # Split by semicolon and parse each key-value pair
        pairs = line.strip().rstrip(';').split(';')
        for pair in pairs:
            if ',' in pair:
                parts = pair.split(',')
                if len(parts) >= 2:
                    key = parts[0].strip()
                    value = parts[1].strip()
                    # Try to convert to int, skip if it fails (for non-numeric fields like 'bpm', '%', etc.)
                    try:
                        data[key] = int(value)
                    except ValueError:
                        # Skip non-integer values
                        pass
        
        # Verify we have all expected keys
        expected_keys = ['GREEN1', 'GREEN2', 'IR1', 'IR2', 'RED1', 'RED2', 'X', 'Y', 'Z']
        if all(key in data for key in expected_keys):
            return data
    except (ValueError, AttributeError) as e:
        log.debug(f"Failed to parse line: {line.strip()} - {e}")
    
    return None


def init_jlink():
    """Initialize JLink connection and RTT"""
    global jlink, start_time
    
    jlink = pylink.JLink()
    logging.getLogger("pylink.jlink").setLevel(logging.WARNING)
    
    log.info("Connecting to JLink...")
    if SERIAL_NUMBER:
        jlink.open(serial_no=SERIAL_NUMBER)
    else:
        jlink.open()
    
    log.info(f"Connecting to {TARGET_DEVICE}...")
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(TARGET_DEVICE)
    jlink.rtt_start(None)
    
    start_time = current_milli_time()
    log.info("RTT connection established. Starting data acquisition...")


def read_rtt_data():
    """Read data from RTT and parse it - write to circular buffer"""
    global line_buffer, sample_count, write_idx
    
    if not jlink or not jlink.connected():
        return 0
    
    new_samples = 0
    try:
        # Read larger chunks for efficiency
        read_bytes = jlink.rtt_read(0, 4096)
        
        if read_bytes:
            data = "".join(map(chr, read_bytes))
            line_buffer += data
            
            # Process all complete lines
            lines = line_buffer.split('\n')
            line_buffer = lines[-1]  # Keep incomplete line
            
            for line in lines[:-1]:
                parsed_data = parse_data_line(line)
                
                if parsed_data:
                    # Calculate elapsed time in seconds
                    elapsed_time = (current_milli_time() - start_time) / 1000.0
                    
                    # Write to circular buffer (in-place update, no allocation)
                    idx = write_idx % MAX_POINTS
                    buffers['time'][idx] = elapsed_time
                    buffers['GREEN1'][idx] = parsed_data['GREEN1']
                    buffers['IR1'][idx] = parsed_data['IR1']
                    buffers['RED1'][idx] = parsed_data['RED1']
                    buffers['GREEN2'][idx] = parsed_data['GREEN2']
                    buffers['IR2'][idx] = parsed_data['IR2']
                    buffers['RED2'][idx] = parsed_data['RED2']
                    buffers['X'][idx] = parsed_data['X']
                    buffers['Y'][idx] = parsed_data['Y']
                    buffers['Z'][idx] = parsed_data['Z']
                    
                    write_idx += 1
                    new_samples += 1
                    sample_count += 1
            
            if new_samples > 0 and sample_count % 200 == 0:
                log.info(f"Samples: {sample_count}, Write idx: {write_idx}")
    
    except Exception as e:
        log.error(f"Error reading RTT data: {e}")
    
    return new_samples


def init_plot():
    """Initialize matplotlib figure with blitting optimization"""
    # Three subplots: GREEN1, GREEN2, Accelerometer
    fig, axes = plt.subplots(3, 1, figsize=(14, 11), gridspec_kw={'height_ratios': [2, 2, 1]})
    fig.suptitle('Heart Rate Sensor - Real-time Raw Data', fontsize=14, fontweight='bold')

    # Create lines with animated=True for blitting
    lines = {}

    # Subplot 0: GREEN1
    axes[0].set_title('GREEN1 (IR/Green depending on setup)', fontsize=10)
    axes[0].set_ylabel('Amplitude')
    axes[0].set_ylim(PPG_Y_MIN, PPG_Y_MAX)
    axes[0].set_xlim(0, WINDOW_SIZE)
    axes[0].set_autoscale_on(False)
    axes[0].grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    lines['GREEN1'], = axes[0].plot([], [], 'b-', label='GREEN1', linewidth=1, animated=True)
    # Info text (top-left inside axes)
    lines['GREEN1_text'] = axes[0].text(0.01, 0.95, '', transform=axes[0].transAxes,
                                      va='top', ha='left', fontsize=9, color='black', animated=True)
    axes[0].legend(loc='upper right', framealpha=0.9)

    # Subplot 1: GREEN2
    axes[1].set_title('GREEN2 (paired with GREEN1)', fontsize=10)
    axes[1].set_ylabel('Amplitude')
    axes[1].set_ylim(PPG_Y_MIN, PPG_Y_MAX)
    axes[1].set_xlim(0, WINDOW_SIZE)
    axes[1].set_autoscale_on(False)
    axes[1].grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    lines['GREEN2'], = axes[1].plot([], [], 'r-', label='GREEN2', linewidth=1, animated=True)
    lines['GREEN2_text'] = axes[1].text(0.01, 0.95, '', transform=axes[1].transAxes,
                                      va='top', ha='left', fontsize=9, color='black', animated=True)
    axes[1].legend(loc='upper right', framealpha=0.9)

    # Subplot 2: Accelerometer
    axes[2].set_title('Accelerometer (X, Y, Z)', fontsize=10)
    axes[2].set_ylabel('Acceleration (mg)')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylim(ACC_Y_MIN, ACC_Y_MAX)
    axes[2].set_xlim(0, WINDOW_SIZE)
    axes[2].set_autoscale_on(False)
    axes[2].grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
    lines['X'], = axes[2].plot([], [], 'r-', label='X', linewidth=1, animated=True)
    lines['Y'], = axes[2].plot([], [], 'g-', label='Y', linewidth=1, animated=True)
    lines['Z'], = axes[2].plot([], [], 'b-', label='Z', linewidth=1, animated=True)
    axes[2].legend(loc='upper right', framealpha=0.9)

    plt.tight_layout()

    # Draw once to establish background
    fig.canvas.draw()

    # Cache backgrounds for each subplot for blitting
    backgrounds = [fig.canvas.copy_from_bbox(ax.bbox) for ax in axes]

    return fig, axes, lines, backgrounds


def update_plot_blit(fig, axes, lines, backgrounds):
    """Update plot with manual blitting - fastest method"""
    # Read new RTT data
    read_rtt_data()
    
    if write_idx == 0:
        return
    
    # Determine how many samples we have
    num_samples = min(write_idx, MAX_POINTS)
    
    if num_samples < 2:
        return
    
    # Extract data from circular buffer in correct order
    if write_idx >= MAX_POINTS:
        # Buffer has wrapped - reorganize to get chronological order
        start_idx = write_idx % MAX_POINTS
        # Data from start_idx to end, then from 0 to start_idx gives chronological order
        time_view = np.concatenate([buffers['time'][start_idx:], buffers['time'][:start_idx]])
        ppg1_view = np.concatenate([buffers['GREEN1'][start_idx:], buffers['GREEN1'][:start_idx]])
        ppg4_view = np.concatenate([buffers['GREEN2'][start_idx:], buffers['GREEN2'][:start_idx]])
        accx_view = np.concatenate([buffers['X'][start_idx:], buffers['X'][:start_idx]])
        accy_view = np.concatenate([buffers['Y'][start_idx:], buffers['Y'][:start_idx]])
        accz_view = np.concatenate([buffers['Z'][start_idx:], buffers['Z'][:start_idx]])
    else:
        # Buffer not yet full - simple slice
        time_view = buffers['time'][:num_samples]
        ppg1_view = buffers['GREEN1'][:num_samples]
        ppg4_view = buffers['GREEN2'][:num_samples]
        accx_view = buffers['X'][:num_samples]
        accy_view = buffers['Y'][:num_samples]
        accz_view = buffers['Z'][:num_samples]
    
    # Get current time for window calculation
    current_time = time_view[-1]
    
    # Always show the last WINDOW_SIZE seconds of data
    if current_time < WINDOW_SIZE:
        # Beginning: not enough data yet, show all data starting from 0
        time_display = time_view
        ppg1_display = ppg1_view
        ppg4_display = ppg4_view
        accx_display = accx_view
        accy_display = accy_view
        accz_display = accz_view
    else:
        # Normal: show last WINDOW_SIZE seconds, shifted to start at 0
        window_start = current_time - WINDOW_SIZE
        mask = time_view >= window_start
        time_display = time_view[mask] - window_start
        
        ppg1_display = ppg1_view[mask]
        ppg4_display = ppg4_view[mask]
        accx_display = accx_view[mask]
        accy_display = accy_view[mask]
        accz_display = accz_view[mask]
    
    # If buffer can't yet cover a full window, warn once
    # Estimate effective rate: samples/time
    if time_view[-1] > 0:
        effective_rate = num_samples / time_view[-1]
        needed = int(WINDOW_SIZE * max(1.0, effective_rate))
        if needed > MAX_POINTS and (sample_count % 500 == 0):
            log.warning(
                f"MAX_POINTS={MAX_POINTS} may be too small for WINDOW_SIZE={WINDOW_SIZE}s "
                f"at ~{effective_rate:.0f} Hz. Consider increasing to >= {needed}"
            )

    # Downsample for drawing to keep performance high
    def decimate(x, y, nmax):
        if len(x) <= nmax:
            return x, y
        step = max(1, len(x) // nmax)
        return x[::step], y[::step]

    td, y1 = decimate(time_display, ppg1_display, DRAW_MAX_POINTS)
    lines['GREEN1'].set_data(td, y1)
    td, y4 = decimate(time_display, ppg4_display, DRAW_MAX_POINTS)
    lines['GREEN2'].set_data(td, y4)
    td, y = decimate(time_display, accx_display, DRAW_MAX_POINTS)
    lines['X'].set_data(td, y)
    td, y = decimate(time_display, accy_display, DRAW_MAX_POINTS)
    lines['Y'].set_data(td, y)
    td, y = decimate(time_display, accz_display, DRAW_MAX_POINTS)
    lines['Z'].set_data(td, y)
    
    # Optional: periodic autoscale for PPG to keep signal visible
    global last_autoscale_time
    now = time.monotonic()
    if now - last_autoscale_time >= PPG_AUTOSCALE_EVERY_S and len(time_display) > 2:
        # Autoscale GREEN1 subplot
        ymin1 = float(np.min(y1)) if len(y1) else PPG_Y_MIN
        ymax1 = float(np.max(y1)) if len(y1) else PPG_Y_MAX
        span1 = max(PPG_MIN_SPAN, ymax1 - ymin1)
        pad1 = max(1.0, span1 * PPG_Y_PADDING_FRACTION)
        axes[0].set_ylim(ymin1 - pad1, ymin1 + span1 + pad1)

        # Autoscale GREEN2 subplot
        ymin4 = float(np.min(y4)) if len(y4) else PPG_Y_MIN
        ymax4 = float(np.max(y4)) if len(y4) else PPG_Y_MAX
        span4 = max(PPG_MIN_SPAN, ymax4 - ymin4)
        pad4 = max(1.0, span4 * PPG_Y_PADDING_FRACTION)
        axes[1].set_ylim(ymin4 - pad4, ymin4 + span4 + pad4)

        # Refresh backgrounds for both PPG axes
        fig.canvas.draw()
        backgrounds[0] = fig.canvas.copy_from_bbox(axes[0].bbox)
        backgrounds[1] = fig.canvas.copy_from_bbox(axes[1].bbox)
        last_autoscale_time = now

    # Update info text for GREEN1 and GREEN2
    if len(y1):
        p2p1 = float(np.max(y1) - np.min(y1))
        txt1 = f"min={np.min(y1):.0f}  max={np.max(y1):.0f}  p2p={p2p1:.0f}  mean={np.mean(y1):.0f}"
        lines['GREEN1_text'].set_text(txt1)
    else:
        lines['GREEN1_text'].set_text('')

    if len(y4):
        p2p4 = float(np.max(y4) - np.min(y4))
        txt4 = f"min={np.min(y4):.0f}  max={np.max(y4):.0f}  p2p={p2p4:.0f}  mean={np.mean(y4):.0f}"
        lines['GREEN2_text'].set_text(txt4)
    else:
        lines['GREEN2_text'].set_text('')

    # Blit each subplot
    for i, (ax, bg) in enumerate(zip(axes, backgrounds)):
        fig.canvas.restore_region(bg)
        
        # Draw only the animated artists in this subplot
        if i == 0:
            ax.draw_artist(lines['GREEN1'])
            ax.draw_artist(lines['GREEN1_text'])
        elif i == 1:
            ax.draw_artist(lines['GREEN2'])
            ax.draw_artist(lines['GREEN2_text'])
        elif i == 2:
            ax.draw_artist(lines['X'])
            ax.draw_artist(lines['Y'])
            ax.draw_artist(lines['Z'])
        
        fig.canvas.blit(ax.bbox)
    
    fig.canvas.flush_events()


def cleanup():
    """Cleanup resources"""
    global jlink
    if jlink and jlink.connected():
        log.info("Closing JLink connection...")
        jlink.close()
    log.info("Cleanup complete.")


def main():
    """Main function - uses manual update loop for maximum performance"""
    try:
        # Initialize JLink and RTT
        init_jlink()
        
        # Initialize plot
        fig, axes, lines, backgrounds = init_plot()
        
        log.info("Starting real-time plot. Close the window to exit.")
        
        # Manual update loop with blitting (fastest method)
        plt.show(block=False)
        plt.pause(0.001)
        
        try:
            while plt.fignum_exists(fig.number):
                update_plot_blit(fig, axes, lines, backgrounds)
                plt.pause(0.001)  # Yield to GUI, minimal delay
        except KeyboardInterrupt:
            log.info("Interrupted by user.")
        
    except Exception as e:
        log.error(f"Error: {e}", exc_info=True)
    finally:
        cleanup()


if __name__ == "__main__":
    main()
