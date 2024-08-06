# Code that logs voltage, and plots the data once logging is stopped

import serial
import matplotlib.pyplot as plt

def open_serial_port(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate)
        print(f"Opened serial port {port} with baud rate {baud_rate}")
        return ser
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return None

def read_from_serial(ser):
    try:
        byte_data = ser.read(1)
        return byte_data
    except Exception as e:
        print(f"Error reading from serial port: {e}")
        return None

def convert_to_ascii_decimal(byte_data):
    return ord(byte_data)

def log_to_file(log_file, data):
    try:
        log_file.write(f"{data}\n")
        log_file.flush()
    except Exception as e:
        print(f"Error writing to file: {e}")

def wait_for_end_of_boot_message(ser, end_marker=b'\n'):
    """
    Reads from serial until the end_marker is found.
    
    Args:
    - ser (serial.Serial): The serial port object.
    - end_marker (bytes): The marker indicating the end of the boot message (default is LF).
    
    Returns:
    - bool: True if the end marker is found, False otherwise.
    """
    while True:
        byte_data = read_from_serial(ser)
        if byte_data is None:
            return False
        if byte_data == end_marker:
            return True
        
def read_data_from_file(file_path):
    """
    Reads ASCII decimal values from a file.
    
    Args:
    - file_path (str): The path to the log file.
    
    Returns:
    - data (list of int): A list of ASCII decimal values.
    """
    data = []
    try:
        with open(file_path, 'r') as file:
            for line in file:
                try:
                    data.append(int(line.strip()))
                except ValueError:
                    print(f"Could not convert line to integer: {line.strip()}")
    except FileNotFoundError:
        print(f"File not found: {file_path}")
    return data

def filter_data(file_path,file_1,file_2):
    """
    Filters the data from the log file and writes the filtered data to a new file.
    
    Args:
    - file_path (str): The path to the log file.
    """
    data = read_data_from_file(file_path)
    if data:
        with open(file_1, 'w') as file1:
            with open(file_2, 'w') as file2:
                for i in range(0, len(data),2):
                    if data[i] == 48:
                        file1.write(f"{data[i+1]}\n")
                    else:
                        file2.write(f"{data[i+1]}\n")

def plot_data(data, interval=1):
    """
    Plots the ASCII decimal values using time in seconds as the x-axis.
    
    Args:
    - data (list of int): A list of ASCII decimal values.
    - interval (int): The time interval between data points in seconds.
    """
    # Create the time values for the x-axis
    time = [i * interval for i in range(len(data))]

    plt.figure(figsize=(10, 5))
    plt.plot(time, data, linestyle='-', color='b')
    plt.ylim(0, 100)
    plt.title('Battery discharge over time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Battery percentage')
    plt.grid(True)
    plt.show()

def main():
    serial_port = '/dev/ttyACM1'  # Replace with your port
    baud_rate = 115200
    log_file_path = 'serial_log.txt'
    file_1='freebot_1.txt'
    file_2='freebot_2.txt'
    end_marker = b'\n'  # ASCII LF character

    # Open the serial port
    ser = open_serial_port(serial_port, baud_rate)
    if ser is None:
        return

    # If you don't need to wait for the end of the boot message, you can remove this block (61-66)
    # Wait for the end of the boot message
    print("Waiting for the end of the boot message...")
    if not wait_for_end_of_boot_message(ser, end_marker):
        print("End marker not found. Exiting.")
        ser.close()
        return
    print("End marker found. Logging started.")

    # Open the file to log the data
    with open(log_file_path, 'w') as log_file:
        try:
            while True:
                byte_data = read_from_serial(ser)
                if byte_data:
                    ascii_decimal = convert_to_ascii_decimal(byte_data)
                    log_to_file(log_file, ascii_decimal)
        except KeyboardInterrupt:
            print("Logging stopped by user.")
        finally:
            ser.close()
            print("Serial port closed.")

    filter_data(log_file_path,file_1,file_2)

    file_path_1= 'freebot_1.txt'  
    data_1= read_data_from_file(file_path_1)
    if data_1:
        plot_data(data_1)
        
    file_path_2= 'freebot_2.txt'  
    data_2= read_data_from_file(file_path_2)
    if data_2:
        plot_data(data_2)

if __name__ == "__main__":
    main()