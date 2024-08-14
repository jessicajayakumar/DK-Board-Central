# GUI for controlling the FreeBot by sending commands from the DKB
# Able to log voltage readings from the FreeBot and plot the data
# Author: Jessica
# Date: 2024-08-14

import sys
import tkinter as tk
import tkinter.ttk as ttk
from tkinter.constants import *
import os.path

import matplotlib.pyplot as plt
import serial

_location = os.path.dirname(__file__)

serial_port = '/dev/ttyACM2'  # Replace with your port
baud_rate = 115200
log_file_path = 'serial_log.txt'
file_1='freebot_1.txt'
file_2='freebot_2.txt'
end_marker = b'\n'  # ASCII LF character

_debug = True # False to eliminate debug printing from callback functions.

_bgcolor = '#d9d9d9'
_fgcolor = '#000000'
_tabfg1 = 'black' 
_tabfg2 = 'white' 
_bgmode = 'light' 
_tabbg1 = '#d9d9d9' 
_tabbg2 = 'gray40'

def main(*args):
    '''Main entry point for the application.'''
    global root
    root = tk.Tk()
    root.protocol( 'WM_DELETE_WINDOW' , root.destroy)
    # Creates a toplevel widget.
    global _top1, _w1
    _top1 = root
    _w1 = Toplevel1(_top1)
    root.mainloop()

class Toplevel1:

    def __init__(self, top=None):
        '''This class configures and populates the toplevel window.
           top is the toplevel containing window.'''

        top.geometry("709x545+2825+178")
        top.minsize(1, 1)
        top.maxsize(3825, 1050)
        top.resizable(1,  1)
        top.title("Toplevel 0")

        self.top = top

        self.recipient = None

        self.dkb_instance = DKB(master=None)
        

        self.Frame1 = tk.Frame(self.top)
        self.Frame1.place(relx=0.508, rely=0.055, relheight=0.266, relwidth=0.458)
        self.Frame1.configure(relief='groove',borderwidth="2")

        self.Label1 = tk.Label(self.Frame1)
        self.Label1.place(relx=0.092, rely=0.069, height=31, width=279)
        self.Label1.configure(activebackground="#d9d9d9", anchor='w',compound='left', text='''Select the recepient of the command:''')

        self.boardcast = tk.Button(self.Frame1)
        self.boardcast.place(relx=0.062, rely=0.414, height=71, width=81)
        self.boardcast.configure(activebackground="#d9d9d9",text='''Broadcast''',command=lambda: DKB.set_recipient('broadcast'))

        self.FB1 = tk.Button(self.Frame1)
        self.FB1.place(relx=0.369, rely=0.414, height=71, width=81)
        self.FB1.configure(activebackground="#d9d9d9",font="-family {DejaVu Sans} -size 10",text='''FreeBot 1''',command=lambda: DKB.set_recipient('FB1'))

        self.FB2 = tk.Button(self.Frame1)
        self.FB2.place(relx=0.677, rely=0.414, height=71, width=81)
        self.FB2.configure(activebackground="#d9d9d9",font="-family {DejaVu Sans} -size 10",text='''FreeBot 2''',command=lambda: DKB.set_recipient('FB2'))

        self.Plot = tk.Button(self.top)
        self.Plot.place(relx=0.282, rely=0.055, height=161, width=141)
        self.Plot.configure(activebackground="#d9d9d9",font="-family {DejaVu Sans} -size 10", text='''PLOT / STOP LOG''')

        self.Log = tk.Button(self.top)
        self.Log.place(relx=0.056, rely=0.055, height=161, width=141)
        self.Log.configure(activebackground="#d9d9d9",text='''START LOGGING''')

        self.Start = tk.Button(self.top)
        self.Start.place(relx=0.127, rely=0.404, height=131, width=231)
        self.Start.configure(activebackground="#d9d9d9",text='''Move ON''')

        self.Stop = tk.Button(self.top)
        self.Stop.place(relx=0.127, rely=0.697, height=131, width=231)
        self.Stop.configure(activebackground="#d9d9d9",text='''Move OFF''')

        self.v_stop = tk.Button(self.top)
        self.v_stop.place(relx=0.55, rely=0.697, height=131, width=231)
        self.v_stop.configure(activebackground="#d9d9d9",font="-family {DejaVu Sans} -size 10",text='''Voltage Send OFF''', command=DKB.voltage_send_off)

        self.v_start = tk.Button(self.top)
        self.v_start.place(relx=0.55, rely=0.404, height=131, width=231)
        self.v_start.configure(activebackground="#d9d9d9",font="-family {DejaVu Sans} -size 10",text='''Voltage Send ON''', command=DKB.voltage_send_on)

  
        
class DKB:

    def __init__(self,master=None):
    
        self.ser = self.open_serial_port(serial_port, baud_rate)
        if self.ser is None:
            return
        
    def set_recipient(self,recipient):
        self.recipient = recipient
    
    def voltage_send_on(self):
        print("Voltage send ON button clicked")
        if self.recipient == 'broadcast':
            self.write_to_serial(self.ser, b'99d')
        elif self.recipient == 'FB1':
            self.write_to_serial(self.ser, b'00d')
        elif self. recipient == 'FB2':
            self.write_to_serial(self.ser, b'01d')
        else:
            print("No recipient selected")
    
    def voltage_send_off(self):
        print("Voltage send off button pressed")
        if self.recipient == 'broadcast':
            self.dkb_instance.write_to_serial(self.dkb_instance.ser, b'99c')
        elif self.recipient == 'FB1':
            self.dkb_instance.write_to_serial(self.dkb_instance.ser, b'00c')
        elif self. recipient == 'FB2':
            self.dkb_instance.write_to_serial(self.dkb_instance.ser, b'01c')
        else:
            print("No recipient selected")

    def open_serial_port(self,port, baud_rate):
        try:
            ser = serial.Serial(port, baud_rate)
            print(f"Opened serial port {port} with baud rate {baud_rate}")
            return ser
        except Exception as e:
            print(f"Error opening serial port: {e}")
            return None
        
    # function to write to serial port
    def write_to_serial(self,ser, data):
        try:
            ser.write(data)
        except Exception as e:
            print(f"Error writing to serial port: {e}")

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

    def close_serial_port(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")

    def __del__(self):
        self.close_serial_port()


if __name__ == '__main__':
    main()