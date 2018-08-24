# IMUExample
NetBurner example code for interfacing with an MPU9250 IMU, tested on the NANO54415 module. Basic I2C register
manipulation, initialization, calibration, and data read. Applies a Madgwick Filter on raw data, which is 
updated at a specified rate, to produce roll, pitch, and yaw. For more information, see "Interfacing the MPU9250 IMU for Absolute
Orientation Data" article at netburner.com.
