# IMUExample
NetBurner example code for interfacing with an MPU9250 IMU, tested on the NANO54415 module. Basic I2C register
manipulation, initialization, calibration, and data read. Applies a Madgwick Filter on raw data, which is 
updated at a specified rate, to produce roll, pitch, and yaw.

Article: https://www.netburner.com/learn/interfacing-the-mpu9250-imu-for-absolute-orientation-data/
