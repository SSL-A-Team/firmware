import serial.tools.list_ports
import time

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

try:
    while True:
        bytes_to_read = ser.in_waiting # Check how many bytes are in the buffer
        print(f"reading bytes: {bytes_to_read}")
        if bytes_to_read:
            data = ser.read(bytes_to_read)
            print(data) # Data is in bytes

        # # Read a line from the serial port
        # # The data is returned as a bytes object (e.g., b'data\r\n')
        # line = ser.read()
        # if line:
        #     print(f"Received: {line}")
        #     # You may need to decode the bytes to a string if it's text data
        #     # decoded_line = line.decode('utf-8').strip()
        #     # print(f"Decoded: {decoded_line}")
        
        time.sleep(0.0001) # 100uS, expecting packets every ms

except KeyboardInterrupt:
    print("Program terminated by user")

except serial.SerialException as e:
    print(f"Serial port error: {e}")

finally:
    # Close the serial port
    if ser.is_open:
        ser.close()
        print("Serial port closed")