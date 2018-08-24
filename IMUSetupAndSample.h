/* MPU9250 Basic Functionality
 Most implementation by Kris Winer ; Ported to Netburner 2.8/3.0 by David Linn
 date: 4/1/14 ; 6/8/18

 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick algorithm.
 Designed to run on the Netburner NANO 54415.

 Hardware setup:
 MPU9250 Breakout --------- NANO
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- 29
 SCL ----------------------- 27
 INT ----------------------- 50
 GND ---------------------- GND
 */

#ifndef IMUSETUPANDSAMPLE_H_
#define IMUSETUPANDSAMPLE_H_

//Methods designed to be called from main()
//---Note: "extern MPU9250 imu;" - Make sure an MPU9250 object is initialized
//with global scope before calling these methods
//---Note: some pins and constants are hardcoded
void IMUSetup();
void IMURun();
float getHeading();
float* getQuaternion();
float degreeWrap(float deg);
void zeroHeading();

//Methods designed to be called from IMUSetup() and IMURun()
void IMUSampleLoop(void*);
void CalibAccAndGyro();
void unclodI2C();
int whoAmICheck();

#endif /* IMUSETUPANDLOOP_H_ */
