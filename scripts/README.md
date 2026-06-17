# Scripts Directory

## Torque Data Writer

The `torque_data_writer.py` script decodes and displays telemetry packets from the control board's USB interface. It can either print decoded packets to the console or save them to a JSON file for later analysis.

### Features

- **Automatic C Header Parsing**: Reads struct definitions from `software-communication/ateam-common-packets/include/`
- **Configurable Packet Decoding**: Use command line options to decode different packet types
- **Dual Output Modes**: Print to console or save to JSON file with metadata
- **Smart Output Formatting**: Shows bitfields, enums, nested structs, and arrays in human-readable format
- **Packet Logging**: Records timestamps, packet counts, and raw hex data when saving to file

### Quick Start

1. **Print to Console** (default):
   ```bash
   ./run_torque_writer.sh
   ```

2. **Save to File**:
   ```bash
   ./run_torque_writer.sh --output telemetry_data.json
   ```

3. **Decode Different Struct**:
   ```bash
   ./run_torque_writer.sh --struct MotorTelemetry --output motor_data.json
   ```

4. **Manual Setup**:
   ```bash
   ./setup_env.sh
   source venv/bin/activate
   python torque_data_writer.py --output data.json
   ```

### Command Line Options

- `--output`, `-o`: Output file to save decoded packets (JSON format). If not specified, packets are printed to console.
- `--struct`, `-s`: Struct name to decode (default: `CurrentControlledMotor_Telemetry`)

### Examples

```bash
# Print decoded packets to console
python torque_data_writer.py

# Save packets to file
python torque_data_writer.py --output telemetry.json

# Decode different struct and save
python torque_data_writer.py -s MotorTelemetry -o motor.json
```

### Output File Format

When using `--output`, the script creates a JSON file containing:
```json
{
  "format": "torque_data_writer_output",
  "struct_name": "CurrentControlledMotor_Telemetry",
  "struct_size": 60,
  "start_time": "2026-01-19T10:30:00.123456",
  "end_time": "2026-01-19T10:35:00.123456",
  "total_packets": 150,
  "packets": [
    {
      "packet_number": 1,
      "timestamp": "2026-01-19T10:30:00.123456",
      "decoded": { /* structured data */ }
    }
  ]
}
```

### Available Struct Types

From header files in `software-communication/ateam-common-packets/include/`:
- `CurrentControlledMotor_Telemetry` (default)
- `CurrentControlledMotor_Response`
- `MotorTelemetry`
- `BasicTelemetry`
- `ExtendedTelemetry`
- And others defined in the header files

### Dependencies

- `pyserial>=3.5` - For USB/serial communication
- `matplotlib>=3.5.0` - For plotting telemetry data (used by plot_telemetry.py)
- `numpy>=1.21.0` - For numerical operations (used by plot_telemetry.py)

## Plot Telemetry

The `plot_telemetry.py` script reads JSON files saved by `torque_data_writer.py` and creates plots of the telemetry data.

### Features

- **Automatic Field Detection**: Discovers all numeric fields in the JSON data
- **Nested Struct Support**: Handles nested structures and arrays
- **Multiple Plot Styles**: Choose from different matplotlib styles
- **Statistics Display**: Shows mean, std, min, max for each field
- **Time-based X-axis**: Plots data against actual timestamps
- **Flexible Output**: Display plots or save to file

### Usage

```bash
# Display plots interactively
python plot_telemetry.py telemetry_data.json

# Save plot to file
python plot_telemetry.py telemetry_data.json --output plot.png

# Plot specific fields only
python plot_telemetry.py data.json --fields current.q_current velocity.velocity

# Use different plot style
python plot_telemetry.py data.json --style dark_background
```

### Command Line Options

- `input` (required): JSON file saved by torque_data_writer.py
- `--output`, `-o`: Save plot to file instead of displaying
- `--fields`, `-f`: Specific fields to plot (space-separated list)
- `--style`, `-s`: Matplotlib style (default, seaborn, ggplot, dark_background)

### Examples

```bash
# Basic usage - show all fields
./plot_telemetry.py telemetry.json

# Select specific fields and save
./plot_telemetry.py data.json -f current.i_a current.i_b current.i_c -o currents.png

# Dark theme for presentations
./plot_telemetry.py data.json -s dark_background -o presentation_plot.png
```