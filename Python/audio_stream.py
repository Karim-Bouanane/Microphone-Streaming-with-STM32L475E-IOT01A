import serial
import matplotlib.pyplot as plt
import numpy as np
import struct
from collections import deque
import time

current_milli_time = lambda: int(round(time.time() * 1000))

# Constants
SERIAL_PORT = 'COM5'
BAUD_RATE = 1200000    # bits/s
SAMPLE_DEPTH = 16      # bits
SAMPLE_SIZE = 2        # bytes
NBR_SAMPLES = 1000     # samples

# Data storage
x_data = np.arange(NBR_SAMPLES)
y_data = deque([0] * NBR_SAMPLES, maxlen=NBR_SAMPLES)
nb_recv_samples = 0

# Plot initialization
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(x_data, y_data)
ax.set_ylim(-pow(2, SAMPLE_DEPTH - 1), pow(2, SAMPLE_DEPTH - 1))

def init_serial():
    global ser
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=None)
    ser.flushInput()
    ser.flushOutput()

def read_serial():
    raw_data = ser.read(SAMPLE_SIZE * NBR_SAMPLES)
    if len(raw_data) == (SAMPLE_SIZE * NBR_SAMPLES):
        values = struct.unpack(f"<{NBR_SAMPLES}h", raw_data)
    else:
        values = -1
    return values

def update_plot(values):
    y_data.extend(values)
    line.set_ydata(y_data)
    plt.draw()
    plt.pause(0.03)

if __name__ == '__main__':
    try:
        init_serial()
    except Exception as e:
        print(f"Open Serial Port Error: {e}")
    else:
        prev_time = current_milli_time()
        try:
            while True:
                ser.write(b"Send")
                values = read_serial()
                if values == -1:
                    print("Received incompleted data")
                else:
                    update_plot(values)
                    nb_recv_samples += NBR_SAMPLES

                # calculate samples rate
                if current_milli_time() - prev_time > 1000:
                    print(str(nb_recv_samples) + " samples/s")
                    nb_recv_samples = 0
                    prev_time = current_milli_time()

                # Check if window is closed
                if not plt.fignum_exists(fig.number):
                    break

        except serial.SerialException as e:
            print(f"Serial Port Error: {e}")

        except Exception as e:
            print(f"Error: {e}")

        finally:
            ser.close()
            plt.ioff()
            plt.close()


