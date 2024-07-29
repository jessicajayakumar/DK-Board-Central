import serial

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

def main():
    serial_port = '/dev/ttyACM0'  # Replace with your port
    baud_rate = 115200
    log_file_path = 'serial_log.txt'
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

if __name__ == "__main__":
    main()