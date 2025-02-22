# 9DOF IMU Kalman Filter

This project implements a Kalman Filter for a 9-DOF IMU (MPU9250) to estimate roll, pitch, and yaw angles. 

## Features
- Sensor Fusion: Combines data from the accelerometer, gyroscope, and magnetometer to estimate orientation.
- Kalman Filter: Implements a 6-state Kalman filter for optimal estimation of roll, pitch, and yaw angles.
- Calibration: Includes calibration routines for the accelerometer, gyroscope, and magnetometer.

## Hardware Requirements
- **MPU9250 IMU**: Combines a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer.
- **Arduino Mega 2560**: Used to interface with the MPU9250 and send data to MATLAB.
- **Wiring**: Connect the MPU9250 to the Arduino using I2C.

## Software Requirements
- **MATLAB**: To run the Kalman filter and plot the results.
- **Arduino Support Package for MATLAB**: Install the Arduino support package to interface with the MPU9250.
- **MPU9250 Library**: Ensure the MPU9250 library is installed on the Arduino.

## Code Overview
The code consists of the following key components:
- **Sensor Calibration**: Calibrates the accelerometer, gyroscope, and magnetometer to remove biases.
- **Kalman Filter**: Implements a 6-state Kalman filter to estimate roll, pitch, and yaw angles.
- **Real-Time Plotting**: Displays the estimated Euler angles in real-time using MATLAB plots.
