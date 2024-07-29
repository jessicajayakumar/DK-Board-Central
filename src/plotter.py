# log code from the serial terminal to a file

import csv
import time

import os
import sys
import matplotlib.pyplot as plt

import serial
import matplotlib.pyplot as plt
from datetime import datetime
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

def save_to_csv(filename, timestamps, values):
    """Save the timestamps and values to a CSV file."""
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Timestamp', 'Data Value'])
        for timestamp, value in zip(timestamps, values):
            writer.writerow([timestamp.strftime('%Y-%m-%d %H:%M:%S'), value])

def plot_data(timestamps, values):
    """Plot the data with timestamps on the x-axis and values on the y-axis."""
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, values, marker='o', linestyle='-', color='b')
    plt.xlabel('Time')
    plt.ylabel('Data Value')
    plt.title('Serial Data Over Time')
    plt.xticks(rotation=45)
    plt.ylim(1, 100)  # Set y-axis limits from 1 to 100
    plt.tight_layout()
    plt.show()


def main():
    # Configure your serial port and file settings here
    port = '/dev/ttyACM1'
    baudrate = 115200
    log_filename = 'data_log.csv'


    timestamps = []
    values = []

    ser = initialize_serial_connection(port, baudrate)
    file = open_log_file(log_filename)

    try:
        while True:
            data = read_from_serial(ser)
            if data:
                decimal_values = convert_chars_to_decimal(data)
                current_time = datetime.now()
                timestamps.extend([current_time] * len(decimal_values))
                values.extend(decimal_values)

                # Write data and timestamps to csv file
                write_to_file(log_filename, [current_time.strftime('%Y-%m-%d %H:%M:%S')] + decimal_values)

                
                # Optional: Print data to console
                print(f"Time: {current_time}, Data: {decimal_values}")
                
                # Update plot in real-time
                plt.clf()
                plot_data(timestamps, values)
                plt.pause(0.1)  # Pause to update plot

    except KeyboardInterrupt:
        print("Logging stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()
        file.close()
        print("Serial port and file closed.")
        save_to_csv(log_filename, timestamps, values)  # Save data to CSV
        plt.show()  # Ensure the final plot is displayed

if __name__ == "__main__":
    main()