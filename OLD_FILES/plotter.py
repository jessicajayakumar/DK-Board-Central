import matplotlib.pyplot as plt

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
    file_path = 'serial_log.txt'  # Path to your log file
    data = read_data_from_file(file_path)
    if data:
        plot_data(data)

if __name__ == "__main__":
    main()
