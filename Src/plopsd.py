import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import welch
import pandas as pd

COM_PORT = 'COM5'
BAUD_RATE = 115200
LOG_DURATION = 300  # seconds
CSV_FILENAME = 'imu_data_log.csv'

ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # wait for device reset

print("Logging started...")

# === Data buffers ===
timestamps = []
ax_list, ay_list, az_list = [], [], []
gx_list, gy_list, gz_list = [], [], []

start_time = time.time()

while (time.time() - start_time) < LOG_DURATION:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        parts = line.split(',')
        if len(parts) != 7:
            continue
        try:
            t = float(parts[0])
            ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
            gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
        except ValueError:
            continue

        timestamps.append(t)
        ax_list.append(ax)
        ay_list.append(ay)
        az_list.append(az)
        gx_list.append(gx)
        gy_list.append(gy)
        gz_list.append(gz)

print("Logging complete.")
ser.close()

df = pd.DataFrame({
    'timestamp': timestamps,
    'ax': ax_list,
    'ay': ay_list,
    'az': az_list,
    'gx': gx_list,
    'gy': gy_list,
    'gz': gz_list
})
df.to_csv(CSV_FILENAME, index=False)
print(f"Data saved to {CSV_FILENAME}")

df = pd.read_csv(CSV_FILENAME)
timestamps = df['timestamp'].to_numpy()
ax = df['ax'].to_numpy()
ay = df['ay'].to_numpy()
az = df['az'].to_numpy()
gx = df['gx'].to_numpy()
gy = df['gy'].to_numpy()
gz = df['gz'].to_numpy()

dt = np.mean(np.diff(timestamps))
fs = 1.0 / dt
print(f"Estimated sample rate: {fs:.2f} Hz")

def plot_psd(signal, fs, label, axis):
    f, Pxx = welch(signal, fs=fs, nperseg=1024)
    axis.semilogy(f, Pxx, label=label)
    return f, Pxx

fig, axs = plt.subplots(3, 1, figsize=(12, 12))

# Raw accelerometer
axs[0].plot(timestamps, ax, label='ax')
axs[0].plot(timestamps, ay, label='ay')
axs[0].plot(timestamps, az, label='az')
axs[0].set_title("Raw Accelerometer")
axs[0].set_ylabel("Accel (g)")
axs[0].grid()
axs[0].legend()

# Raw gyroscope
axs[1].plot(timestamps, gx, label='gx')
axs[1].plot(timestamps, gy, label='gy')
axs[1].plot(timestamps, gz, label='gz')
axs[1].set_title("Raw Gyroscope")
axs[1].set_ylabel("Gyro (°/s)")
axs[1].grid()
axs[1].legend()

fig_psd, axs_psd = plt.subplots(2, 1, figsize=(12, 8))

plot_psd(ax, fs, 'ax', axs_psd[0])
plot_psd(ay, fs, 'ay', axs_psd[0])
plot_psd(az, fs, 'az', axs_psd[0])
axs_psd[0].set_title("Accelerometer PSD")
axs_psd[0].set_xlabel("Frequency (Hz)")
axs_psd[0].set_ylabel("Power")
axs_psd[0].grid()
axs_psd[0].legend()

plot_psd(gx, fs, 'gx', axs_psd[1])
plot_psd(gy, fs, 'gy', axs_psd[1])
plot_psd(gz, fs, 'gz', axs_psd[1])
axs_psd[1].set_title("Gyroscope PSD")
axs_psd[1].set_xlabel("Frequency (Hz)")
axs_psd[1].set_ylabel("Power")
axs_psd[1].grid()
axs_psd[1].legend()

plt.tight_layout()
plt.show()

# === Calculate covariance matrices ===
acc_data = np.vstack([ax, ay, az])
gyro_data = np.vstack([gx, gy, gz])

acc_cov = np.cov(acc_data)
gyro_cov = np.cov(gyro_data)

print("Accelerometer covariance matrix:\n", acc_cov)
print("Gyroscope covariance matrix:\n", gyro_cov)
