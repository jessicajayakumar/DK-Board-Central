    def read_from_serial(self,ser):
        try:
            byte_data = ser.read(1)
            return byte_data
        except Exception as e:
            print(f"Error reading from serial port: {e}")
            return None

    def convert_to_ascii_decimal(self,byte_data):
        return ord(byte_data)

    def log_to_file(self,log_file, data):
        try:
            if data < 96:
                log_file.write(f"{data}\n")
                log_file.flush()
        except Exception as e:
            print(f"Error writing to file: {e}")

    def wait_for_end_of_boot_message(self,ser, end_marker=b'\n'):
        while True:
            byte_data = self.read_from_serial(ser)
            if byte_data is None:
                return False
            if byte_data == end_marker:
                return True
            
    def read_data_from_file(self,file_path):
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

    def filter_data(self,file_path,file_1,file_2):
        data = self.read_data_from_file(file_path)
        if data:
            with open(file_1, 'w') as file1:
                with open(file_2, 'w') as file2:
                    for i in range(0, len(data),2):
                        if data[i] == 49:
                            file1.write(f"{data[i+1]}\n")
                        else:
                            file2.write(f"{data[i+1]}\n")

    def plot_data(self,data, interval=1):
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

    def logging_start(self,ser,log_file_path):
        print("Logging started. Press Ctrl+C to stop.")
        with open(log_file_path, 'w') as log_file:
            try:
                while True:
                    byte_data = self.read_from_serial(ser)
                    if byte_data:
                        ascii_decimal = self.convert_to_ascii_decimal(byte_data)
                        self.log_to_file(log_file, ascii_decimal)
            except KeyboardInterrupt:
                print("Logging stopped by user.")

    def logging_stop(self,log_file_path,file_1,file_2):
        print("Logging stopped, plotting data...")
        self.filter_data(log_file_path,file_1,file_2)

        file_path_1= 'freebot_1.txt'  
        data_1= self.read_data_from_file(file_path_1)
        if data_1:
            self.plot_data(data_1)
            
        file_path_2= 'freebot_2.txt'  
        data_2= self.read_data_from_file(file_path_2)
        if data_2:
            self.plot_data(data_2)