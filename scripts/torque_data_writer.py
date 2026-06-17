import serial.tools.list_ports
import time
import os
import argparse
import jsonlines
from datetime import datetime
from packet_decoder import HeaderParser, PacketDecoder

###############################################################################
# Command Line Arguments
###############################################################################

def parse_arguments():
    parser = argparse.ArgumentParser(description="Decode CcmTelemetry packets from control board")
    parser.add_argument("--output", "-o", type=str, help="Output file to save decoded packets (JSON format). If not specified, packets are printed to console.")
    parser.add_argument("--struct", "-s", type=str, default="CcmTelemetry",
                       help="Struct name to decode (default: CcmTelemetry)")
    return parser.parse_args()

###############################################################################
# Configuration
###############################################################################

# Path to the header files (relative to this script)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEADER_DIR = os.path.join(SCRIPT_DIR, "..", "software-communication", "ateam-common-packets", "include")

# Parse command line arguments
args = parse_arguments()

# Struct to decode - can be overridden by command line
DECODE_STRUCT_NAME = args.struct


###############################################################################
# Initialize Parser and Decoder
###############################################################################

print("Loading header files...")
parser = HeaderParser()

# Parse the relevant header files
for header_file in ["stspin_current.h", "stspin.h"]:
    header_path = os.path.join(HEADER_DIR, header_file)
    if os.path.exists(header_path):
        print(f"  Parsing {header_file}...")
        parser.parse_file(header_path)

print(f"\nLoaded {len(parser.structs)} structs and {len(parser.enums)} enums")
print(f"Decoding struct: {DECODE_STRUCT_NAME}")

target_struct = parser.get_struct(DECODE_STRUCT_NAME)
if target_struct:
    print(f"  Size: {target_struct.size} bytes")
    print(f"  Fields: {len(target_struct.fields)}")
else:
    print(f"  ERROR: Struct '{DECODE_STRUCT_NAME}' not found!")

decoder = PacketDecoder(parser)

###############################################################################
# Serial Connection
###############################################################################

control_board_port = None

ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
        print(f"{port}: {desc} [{hwid}]")

        if "USB VID:PID=C0DE:CAFE SER=12345678" in hwid:
            print(f"Found control board on port {port}")
            control_board_port = port

if control_board_port is None:
    print("no control board found")
    exit(0)

ser = serial.Serial(
    port=control_board_port,
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1 # Timeout in seconds for read operations
)

print(f"Connected to {ser.port}")
print(f"\nWaiting for packets (expecting {target_struct.size if target_struct else '?'} bytes)...")

# Initialize output file if specified
jsonl_writer = None
packet_count = 0
if args.output:
    print(f"Saving packets to: {args.output}")
    output_file = open(args.output, 'w')
    jsonl_writer = jsonlines.Writer(output_file)
    # Write file header with metadata
    metadata = {
        "type": "metadata",
        "format": "torque_data_writer_output",
        "struct_name": DECODE_STRUCT_NAME,
        "struct_size": target_struct.size if target_struct else None,
        "start_time": datetime.now().isoformat()
    }
    jsonl_writer.write(metadata)
    output_file.flush()
else:
    print("Printing packets to console (use --output to save to file)")

print()

###############################################################################
# Main Loop
###############################################################################

try:
    while True:
        bytes_to_read = ser.in_waiting
        if bytes_to_read:
            data = ser.read(bytes_to_read)

            if target_struct and len(data) >= (target_struct.size or 0):
                decoded = decoder.decode(DECODE_STRUCT_NAME, data)
                if decoded:
                    packet_count += 1
                    if jsonl_writer:
                        # Save to file as JSON Lines
                        packet_data = {
                            "type": "packet",
                            "packet_number": packet_count,
                            "timestamp": datetime.now().isoformat(),
                            "decoded": decoded
                        }

                        # Append packet as new line
                        jsonl_writer.write(packet_data)
                        output_file.flush()

                        print(f"Packet {packet_count} saved")
                    else:
                        # Print to console
                        print(f"=== Packet {packet_count} ===")
                        decoder.print_decoded(DECODE_STRUCT_NAME, decoded)
                        print()
                else:
                    print(f"Failed to decode packet")
                    print(f"Raw: {data.hex()}")
            else:
                expected = target_struct.size if target_struct else "unknown"
                print(f"Packet size mismatch: got {len(data)}, expected {expected}")
                print(f"Raw: {data.hex()}")

        time.sleep(0.0001) # 100uS, expecting packets every ms

except KeyboardInterrupt:
    print(f"\nProgram terminated by user. Captured {packet_count} packets.")

except serial.SerialException as e:
    print(f"Serial port error: {e}")

finally:
    # Close the output file if open
    if jsonl_writer:
        # Write final metadata with end time
        end_metadata = {
            "type": "metadata",
            "end_time": datetime.now().isoformat(),
            "total_packets": packet_count
        }
        jsonl_writer.write(end_metadata)
        jsonl_writer.close()
        print(f"Saved {packet_count} packets to {args.output}")

    # Close the serial port
    if ser.is_open:
        ser.close()
        print("Serial port closed")
