import serial
import matplotlib.pyplot as plt
from collections import deque

# Set the serial port and baudrate
serial_port = '/dev/serial0'  # Adjust the port based on your Raspberry Pi configuration
baudrate = 115200

# Initialize the serial connection
ser = serial.Serial(serial_port, baudrate)

# Set up the plot
fig, ax = plt.subplots()
x_data = deque(maxlen=50)  # Keep the last 50 data points for the plot
y1_data = deque(maxlen=50)
y2_data = deque(maxlen=50)
y3_data = deque(maxlen=50)

line1, = ax.plot(x_data, y1_data, label='Float 1')
line2, = ax.plot(x_data, y2_data, label='Float 2')
line3, = ax.plot(x_data, y3_data, label='Float 3')

ax.legend()

def update_plot(new_data):
    x, y1, y2, y3 = map(float, new_data.split(','))

    x_data.append(x)
    y1_data.append(y1)
    y2_data.append(y2)
    y3_data.append(y3)

    line1.set_xdata(x_data)
    line1.set_ydata(y1_data)
    line2.set_xdata(x_data)
    line2.set_ydata(y2_data)
    line3.set_xdata(x_data)
    line3.set_ydata(y3_data)
    ax.relim()
    ax.autoscale_view()

    plt.pause(0.01)
    plt.draw()

try:
    while True:
        data = ser.readline().decode('utf-8').strip()
        print(data)  # Optional: Print received data to console
        update_plot(data)

except KeyboardInterrupt:
    print("Script terminated by user.")
    ser.close()