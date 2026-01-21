#!/usr/bin/env python3
"""
Plot Telemetry Data

This script reads JSON files saved by torque_data_writer.py and plots the
telemetry data over time. It automatically detects numeric fields and creates
plots for visualization and analysis.
"""

import argparse
import jsonlines
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
from pathlib import Path


def parse_arguments():
    parser = argparse.ArgumentParser(description="Plot telemetry data from JSON files")
    parser.add_argument("input", type=str, help="Input JSON file from torque_data_writer")
    parser.add_argument("--fields", "-f", type=str, nargs="+",
                       help="Specific fields to plot (if not specified, plots all numeric fields)")
    parser.add_argument("--output", "-o", type=str,
                       help="Save plot to file instead of displaying")
    parser.add_argument("--style", "-s", type=str, default="default",
                       choices=["default", "seaborn", "ggplot", "dark_background"],
                       help="Matplotlib style to use")
    parser.add_argument("--current-samples", action="store_true",
                       help="Plot current_samples_ma array with proper time offsets (50us per sample)")
    return parser.parse_args()


def flatten_dict(d, parent_key='', sep='.'):
    """Flatten nested dictionary structure"""
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        elif isinstance(v, list) and len(v) > 0 and isinstance(v[0], dict):
            # Handle arrays of structs
            for i, item in enumerate(v):
                items.extend(flatten_dict(item, f"{new_key}[{i}]", sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def extract_numeric_fields(packets):
    """Extract all numeric fields from packets"""
    if not packets:
        return {}

    # Flatten first packet to discover fields
    first_decoded = packets[0].get('decoded', {})
    flat = flatten_dict(first_decoded)

    # Find numeric fields
    numeric_fields = {}
    for key, value in flat.items():
        if isinstance(value, (int, float)) and not key.endswith('_name'):
            numeric_fields[key] = []

    # Extract values from all packets
    for packet in packets:
        decoded = packet.get('decoded', {})
        flat = flatten_dict(decoded)
        for key in numeric_fields.keys():
            value = flat.get(key, None)
            if value is not None and isinstance(value, (int, float)):
                numeric_fields[key].append(value)
            else:
                numeric_fields[key].append(np.nan)

    return numeric_fields


def create_timestamp_array(packets):
    """Create array of timestamps from packets"""
    from datetime import datetime

    timestamps = []
    start_time = None

    for packet in packets:
        ts_str = packet.get('timestamp', '')
        if ts_str:
            ts = datetime.fromisoformat(ts_str)
            if start_time is None:
                start_time = ts
            # Convert to seconds from start
            elapsed = (ts - start_time).total_seconds()
            timestamps.append(elapsed)
        else:
            # Use packet number as fallback
            timestamps.append(packet.get('packet_number', 0) * 0.001)  # Assume 1ms packets

    return np.array(timestamps)


def plot_telemetry(data, timestamps, fields_to_plot=None, output_file=None, style="default"):
    """Create plots for telemetry data"""

    # Filter fields if specified
    if fields_to_plot:
        data = {k: v for k, v in data.items() if k in fields_to_plot}

    if not data:
        print("No data to plot!")
        return

    # Apply style
    plt.style.use(style)

    # Calculate layout
    n_fields = len(data)
    n_cols = min(2, n_fields)
    n_rows = (n_fields + n_cols - 1) // n_cols

    fig = plt.figure(figsize=(12, 4 * n_rows))
    gs = GridSpec(n_rows, n_cols, figure=fig, hspace=0.3, wspace=0.3)

    for idx, (field_name, values) in enumerate(sorted(data.items())):
        row = idx // n_cols
        col = idx % n_cols
        ax = fig.add_subplot(gs[row, col])

        values_array = np.array(values)

        # Plot the data
        ax.plot(timestamps, values_array, linewidth=1, alpha=0.8)
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel(field_name)
        ax.set_title(field_name, fontsize=10, fontweight='bold')
        ax.grid(True, alpha=0.3)

        # Add statistics to the plot
        mean_val = np.nanmean(values_array)
        std_val = np.nanstd(values_array)
        min_val = np.nanmin(values_array)
        max_val = np.nanmax(values_array)

        stats_text = f'μ={mean_val:.2f} σ={std_val:.2f}\nmin={min_val:.2f} max={max_val:.2f}'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
               verticalalignment='top', fontsize=8,
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    fig.suptitle('Telemetry Data', fontsize=14, fontweight='bold')

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {output_file}")
    else:
        plt.show()

    plt.close()


def plot_current_samples(packets, output_file=None, style="default"):
    """Plot current_samples_ma arrays with proper time offsets"""
    from datetime import datetime

    print("Plotting current samples...")

    # Apply style
    plt.style.use(style)

    fig, ax = plt.subplots(figsize=(14, 6))

    # Constants
    SAMPLE_PERIOD_US = 50  # 50 microseconds between samples
    SAMPLE_PERIOD_S = SAMPLE_PERIOD_US * 1e-6  # Convert to seconds

    # Collect all samples with their absolute times
    all_times = []
    all_values = []

    for packet in packets:
        decoded = packet.get('decoded', {})
        current_telemetry = decoded.get('current_telemetry', {})
        current_samples = current_telemetry.get('current_samples_ma')
        timestamp_str = packet.get('timestamp')

        if current_samples is None or timestamp_str is None:
            continue

        # Parse timestamp
        try:
            ts = datetime.fromisoformat(timestamp_str)
        except:
            continue

        # Convert to seconds since epoch
        packet_time = ts.timestamp()

        # Number of samples
        n_samples = len(current_samples)

        # Create time array: last sample is at packet_time, work backwards
        # times[i] = packet_time - (n_samples - 1 - i) * SAMPLE_PERIOD_S
        sample_times = packet_time - np.arange(n_samples - 1, -1, -1) * SAMPLE_PERIOD_S

        all_times.extend(sample_times)
        all_values.extend(current_samples)

    if not all_times:
        print("No current_samples_ma data found in packets")
        return

    # Convert to numpy arrays and sort by time
    all_times = np.array(all_times)
    all_values = np.array(all_values)

    # Sort by time (should already be mostly sorted, but ensure it)
    sort_idx = np.argsort(all_times)
    all_times = all_times[sort_idx]
    all_values = all_values[sort_idx]

    # Convert to relative time (seconds from first sample)
    relative_times = all_times - all_times.min()

    # Plot as single continuous line
    ax.plot(relative_times, all_values, linewidth=0.8, alpha=0.8)

    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Current (mA)', fontsize=12)
    ax.set_title('Current Samples (50μs per sample)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # Add statistics
    mean_val = np.nanmean(all_values)
    std_val = np.nanstd(all_values)
    min_val = np.nanmin(all_values)
    max_val = np.nanmax(all_values)

    stats_text = f'μ={mean_val:.2f} mA\nσ={std_val:.2f} mA\nmin={min_val:.2f} mA\nmax={max_val:.2f} mA\nSamples: {len(all_values)}'
    ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
           verticalalignment='top', fontsize=10,
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {output_file}")
    else:
        plt.show()

    plt.close()


def print_summary(metadata, data):
    """Print summary statistics"""
    print("\n" + "="*70)
    print("TELEMETRY DATA SUMMARY")
    print("="*70)
    print(f"Format: {metadata.get('format', 'unknown')}")
    print(f"Struct: {metadata.get('struct_name', 'unknown')}")
    print(f"Struct Size: {metadata.get('struct_size', 'unknown')} bytes")
    print(f"Total Packets: {metadata.get('total_packets', len(metadata.get('packets', [])))}")
    print(f"Start Time: {metadata.get('start_time', 'unknown')}")
    print(f"End Time: {metadata.get('end_time', 'unknown')}")

    print(f"\nNumeric Fields Found: {len(data)}")
    for field_name, values in sorted(data.items()):
        values_array = np.array(values)
        print(f"  {field_name}:")
        print(f"    Mean: {np.nanmean(values_array):.4f}")
        print(f"    Std:  {np.nanstd(values_array):.4f}")
        print(f"    Min:  {np.nanmin(values_array):.4f}")
        print(f"    Max:  {np.nanmax(values_array):.4f}")
    print("="*70 + "\n")


def main():
    args = parse_arguments()

    # Load JSON Lines file
    print(f"Loading data from {args.input}...")
    try:
        with jsonlines.open(args.input) as reader:
            lines = list(reader)
    except FileNotFoundError:
        print(f"Error: File '{args.input}' not found")
        return
    except Exception as e:
        print(f"Error: Failed to read JSON Lines file - {e}")
        return

    # Separate metadata and packets
    metadata = {}
    packets = []

    for line in lines:
        if line.get('type') == 'metadata':
            metadata.update(line)
        elif line.get('type') == 'packet':
            packets.append(line)

    if not packets:
        print("Error: No packets found in JSON Lines file")
        return

    print(f"Loaded {len(packets)} packets")

    # Check for current samples mode
    if args.current_samples:
        print("Current samples mode enabled")
        plot_current_samples(packets, args.output, args.style)
        print("Done!")
        return

    # Extract numeric fields
    print("Extracting numeric fields...")
    data = extract_numeric_fields(packets)

    if not data:
        print("Error: No numeric fields found to plot")
        return

    # Create timestamps
    timestamps = create_timestamp_array(packets)

    # Print summary
    print_summary(metadata, data)

    # Create plots
    print("Generating plots...")
    plot_telemetry(data, timestamps, args.fields, args.output, args.style)

    print("Done!")


if __name__ == "__main__":
    main()
