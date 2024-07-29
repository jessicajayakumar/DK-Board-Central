# log code from the serial terminal to a file

import time
import os
import sys
import serial
import time


def initialize_serial_connection(port, baudrate, timeout=1):
    """Initialize the serial connection."""
    return serial.Serial(port, baudrate, timeout=timeout)

def open_log_file(filename):
    """Open a file for logging data."""
    return open(filename, 'w')

def read_from_serial(ser):
    """Read a line from the serial port."""
    if ser.in_waiting > 0:
        return ser.readline().decode('utf-8').rstrip()
    return None

def convert_chars_to_decimal(data):
    """Convert a string of characters to their decimal ASCII values."""
    return [str(ord(char)) for char in data]

def write_to_file(file, data):
    """Write data to the log file."""
    file.write(' '.join(data) + '\n')
    file.flush()

def main():
    # Configure your serial port and file settings here
    port = '/dev/ttyACM0'  # Replace with your serial port
    baudrate = 115200
    log_filename = 'voltage.txt'

    ser = initialize_serial_connection(port, baudrate)
    file = open_log_file(log_filename)

    try:
        while True:
            data = read_from_serial(ser)
            if data:
                decimal_values = convert_chars_to_decimal(data)
                print(' '.join(decimal_values))  # Optional: Print to console
                write_to_file(file, decimal_values)
    except KeyboardInterrupt:
        print("Logging stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()
        file.close()
        print("Serial port and file closed.")

if __name__ == "__main__":
    main()