import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

COM_PORT = 'COM5'
BAUD_RATE = 115200
WINDOW_SIZE_SECONDS = 10

ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
ser.flush()

timestamps = []
pitch_gyro_data = []
filtered_gy_data = []
pitch_acc_data = []

# === Plot Setup ===
fig, ax = plt.subplots(figsize=(10, 5))
line_pitch = ax.plot([], [], label='Pitch (Gyro)', color='blue')[0]
line_filtered_gy = ax.plot([], [], label='Filtered Gyro Y', color='red')[0]
line_pitch_acc = ax.plot([], [], label='Pitch (Accel)', color='green')[0]

ax.set_ylabel('Degrees')
ax.set_xlabel('Time (s)')
ax.set_title('Pitch Gyro vs Filtered Gyro Y vs Pitch Accel')
ax.legend()
ax.grid()

start_time = None

def init():
    ax.set_xlim(0, WINDOW_SIZE_SECONDS)
    ax.set_ylim(-180, 180)
    return line_pitch, line_filtered_gy, line_pitch_acc

def update(frame):
    global start_time

    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        parts = line.split(',')
        if len(parts) != 5:
            continue
        try:
            t = float(parts[0])
            pitch_gyro = float(parts[1])
            filtered_gy = float(parts[2])
            pitch_acc = float(parts[3])
        except ValueError:
            continue

        if start_time is None:
            start_time = t
        t -= start_time

        timestamps.append(t)
        pitch_gyro_data.append(pitch_gyro)
        filtered_gy_data.append(filtered_gy)
        pitch_acc_data.append(pitch_acc)

        while timestamps and (t - timestamps[0]) > WINDOW_SIZE_SECONDS:
            timestamps.pop(0)
            pitch_gyro_data.pop(0)
            filtered_gy_data.pop(0)
            pitch_acc_data.pop(0)

    if timestamps:
        ax.set_xlim(timestamps[0], timestamps[-1])
        line_pitch.set_data(timestamps, pitch_gyro_data)
        line_filtered_gy.set_data(timestamps, filtered_gy_data)
        line_pitch_acc.set_data(timestamps, pitch_acc_data)

    return line_pitch, line_filtered_gy, line_pitch_acc

ani = animation.FuncAnimation(fig, update, init_func=init, interval=50, blit=True)
plt.tight_layout()
plt.show()
