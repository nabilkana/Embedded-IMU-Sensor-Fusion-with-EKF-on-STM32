import pandas as pd
import numpy as np

# Load CSV (assumes headers: ax, ay, az, gx, gy, gz)
df = pd.read_csv("imu_data_log.csv")

signals = {
    "ax": df["ax"].values,
    "ay": df["ay"].values,
    "az": df["az"].values,
    "gx": df["gx"].values,
    "gy": df["gy"].values,
    "gz": df["gz"].values,
}

print("Mean and Variance of IMU axes:")
for label, data in signals.items():
    mean_val = np.mean(data)
    var_val = np.var(data)
    print(f"{label}: mean = {mean_val:.6e}, variance = {var_val:.6e}")
