IMU Sensor Driver and Orientation Estimation with EKF on STM32
Overview
This project implements a driver for the MPU-6500 IMU sensor on an STM32 microcontroller to read raw accelerometer and gyroscope data. It includes:

Reading and combining raw 6-axis sensor data via I2C.

Applying filtering techniques: low-pass filter and complementary filter.

Implementing an Extended Kalman Filter (EKF) to estimate roll and pitch angles.

Handling limitations such as gyro drift and accelerometer noise.

Demonstrating gimbal lock effects and discussing mitigation strategies.

Plotting sensor data and filter outputs using Python.

Hardware
STM32 microcontroller (specify model if you want)

MPU-6500 IMU sensor connected via I2C

UART connection for debugging and data transmission

Software
STM32 firmware for sensor initialization, reading, and filtering

Python scripts for logging data from UART, plotting raw and filtered sensor data, and calculating PSDs

Setup Instructions
Flash the STM32 firmware onto your board.

Connect MPU-6500 sensor properly via I2C.

Connect UART interface to your PC.

Use a terminal program (e.g., PuTTY) to receive data at 115200 baud.

Run the Python scripts in the src/ folder to log and visualize data.

Repository Structure
src/ — Python scripts for plotting and logging

stm32/ — STM32 firmware source code including the MPU-6500 driver and filter implementations

