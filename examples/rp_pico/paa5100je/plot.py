import serial
import re
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

pattern = re.compile(r"X: (-?[0-9]+) Y: (-?[0-9]+) dX: (-?[0-9]+) dY: (-?[0-9]+)")
ser = serial.Serial("/dev/ttyACM0", 115200)
x_vals = []
y_vals = []
dx = []
dy = []

def animate(i):
    global x_vals
    global y_vals
    global dx
    global dy
    while ser.in_waiting:
        data = ser.readline().decode().strip()
        match = re.match(pattern, data)
        if match is not None:
            point = (-int(match.group(1)), int(match.group(2)))
            delta = (-int(match.group(3)), int(match.group(4)))
            x_vals.append(point[0])
            y_vals.append(point[1])
            dx.append(delta[0])
            dy.append(delta[1])
        else:
            x_vals = []
            y_vals = []
            dy = []
            dx = []
    plt.cla()
    plt.plot(x_vals,y_vals)
    plt.tight_layout()

def on_close(event):
    sys.exit(0)

plt.ion()
plt.style.use("fivethirtyeight")

fig, axs = plt.subplots(nrows=1, ncols=1, constrained_layout=True)
fig.canvas.mpl_connect("close_event", on_close)

ani = FuncAnimation(fig, animate, interval=100)
while True:
    plt.legend()
    plt.tight_layout()
    plt.pause(0.2)



