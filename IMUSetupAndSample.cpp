/* MPU9250
 Most implementation by Kris Winer ; Ported to NetBurner 2.8.6 with added features by David Linn
 date: 4/1/14 ; 6/8/18
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out.
 9 DoF sensor fusion using open source Madgwick algorithm.
 Tested on the NANO54415 with NetBurner 2.8.6.
 
 Hardware setup:
 MPU9250 Breakout --------- NANO
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- 29
 SCL ----------------------- 27
 INT ----------------------- 50
 GND ---------------------- GND
 */

#include "IMUSetupAndSample.h"
#include "MadgwickFilter.h"
#include "MPU9250.h"
#include "Boot_Settings.h"

//System includes
#include <predef.h>
#if (NNDK_MAJOR == 2)
#include <ucos.h>
#elif (NNDK_MAJOR == 3)
#include <rtos.h>
#else
#error Check NNDK version compatibility with IMU code.
#endif
#include <multichanneli2c.h>
#include <constants.h>
#include <stdio.h>
#include <nbtime.h>
#include <basictypes.h>
#include <pins.h>
#include <pin_irq.h>
#include <math.h>
#include <SimpleAD.h>
#include <system.h>
#include <pitr_sem.h>

#define SerialDebug 1
#define CALIBGA 1 //set to 1 to recalibrate gyro and accelerometer- it's fast so I always keep this on
#define RECALIB_MAG 1 //set to 1 to recalibrate magnetometer- it's slow so I often turn this off after a good calibration
#define IMU_PIT_TIMER 1 //coordinate with other Programmable Interrupt Timers being used
#define MAG_DECL 11.52 // Declination in San Diego, California is 11.52E on 6/8/18
#define UPDATE_HZ 66 //Update rate in Hz
#define IMU_TASK_PRIO (MAIN_PRIO-1)

// Specify sensor full scale
uint8_t Gscale = GFS250DPS;
uint8_t Ascale = AFS2G;
uint8_t Mscale = MFS16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
  
// Pin definitions
volatile bool newData = false;
bool newMagData = false;

int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};      // Bias corrections for gyro and accelerometer
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = M_PI * (4.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = M_PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = 4* sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

int sumCount = 0;
float pitch, yaw, roll, heading;
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
double currenttime, deltat, lasttime, sum;  //for timing integration interval
float zeroOffset; //offset created by zeroing heading

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

extern MPU9250 imu; //Make sure global imu object gets initialized before calling IMUSetup()

void IMUSetup() {
	if (whoAmICheck() == -1) return; //Fail initialization if whoAmICheck fails
	imu.initMPU9250(Ascale,Gscale);
	// get sensor resolutions, only need to do this once
	aRes = imu.getAres(Ascale);
	gRes = imu.getGres(Gscale);
	mRes = imu.getMres(Mscale);
	if (CALIBGA) CalibAccAndGyro();
	imu.initMPU9250(Ascale,Gscale);
	printf("\nMPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	BYTE d = imu.readByte(AK8963_ADDRESS, 0x00);  // Read WHO_AM_I register for AK8963
	printf("\nAK8963 "); printf("I AM "); printf("%x",d); printf(" I should be "); printf("0x48");
	OSTimeDly(TICKS_PER_SECOND);
  
	// Get magnetometer calibration from AK8963 ROM
	imu.initAK8963(Mscale,Mmode,magCalibration); printf("\nAK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
	if (RECALIB_MAG || NV_Settings.magScale[0]==0) { //if recalibMag set or mag calib is 0
		printf("Recalib Mag, move IMU in figure-8 motion for 15 secs...");
		imu.magcalMPU9250(magBias, magScale);
		//Save calibration to boot memory
		for (int i = 0; i < 3; ++i)
			NV_Settings.magBias[i] = magBias[i];
		for (int i = 0; i < 3; ++i)
			NV_Settings.magScale[i] = magScale[i];
		SaveUserParameters(&NV_Settings, sizeof(NV_Settings));
		printf("Mag calib done!\n");
	}
	else {
		//Load calibration from boot memory
		for (int i = 0; i < 3; ++i)
			magBias[i]=NV_Settings.magBias[i];
		for (int i = 0; i < 3; ++i)
			magScale[i]=NV_Settings.magScale[i];
	}
	//Print Mag Calibration
	printf("\nAK8963 mag biases (mG)"); printf(" %f",magBias[0]); printf(" %f",magBias[1]); printf(" %f",magBias[2]);
	printf("\nAK8963 mag scale (mG)"); printf(" %f",magScale[0]); printf(" %f",magScale[1]); printf(" %f",magScale[2]);
	if(SerialDebug) {
		printf("\nX-Axis sensitivity adjustment value "); printf("%f",magCalibration[0]);
		printf("\nY-Axis sensitivity adjustment value "); printf("%f",magCalibration[1]);
		printf("\nZ-Axis sensitivity adjustment value "); printf("%f\n",magCalibration[2]);
		OSTimeDly(TICKS_PER_SECOND);
	}
}

void CalibAccAndGyro() {
	imu.SelfTest(SelfTest); // Start by performing self test and reporting values
	printf("x-axis self test: acceleration trim within : "); printf("%f",SelfTest[0]); printf("percent of factory value");
	printf("\ny-axis self test: acceleration trim within : "); printf("%f",SelfTest[1]); printf("percent of factory value");
	printf("\nz-axis self test: acceleration trim within : "); printf("%f",SelfTest[2]); printf("percent of factory value");
	printf("\nx-axis self test: gyration trim within : "); printf("%f",SelfTest[3]); printf("percent of factory value");
	printf("\ny-axis self test: gyration trim within : "); printf("%f",SelfTest[4]); printf("percent of factory value");
	printf("\nz-axis self test: gyration trim within : "); printf("%f",SelfTest[5]); printf("percent of factory value");
	OSTimeDly(TICKS_PER_SECOND);

	printf("\n Calibrate gyro and accel, keep still");
	imu.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	printf("\naccel biases (mg)"); printf(" %f",1000.*accelBias[0]); printf(" %f",1000.*accelBias[1]); printf(" %f",1000.*accelBias[2]);
	printf("\ngyro biases (dps)"); printf(" %f",gyroBias[0]); printf(" %f",gyroBias[1]); printf(" %f",gyroBias[2]);
	//OSTimeDly(TICKS_PER_SECOND);
}

void IMUSampleLoop(void*) {
	HiResTimer* imuTimer = imu.getIMUTimer();
	imuTimer->start();
	OS_SEM IMUSem;
	InitPitOSSem(IMU_PIT_TIMER, &IMUSem, UPDATE_HZ); //Interrupts UPDATE_HZ times per second
	while (1) {
		IMUSem.Pend();
		imu.readMPU9250Data(MPU9250Data); // interrupt cleared (goes low) on any read
		//   readAccelData(accelCount);  // Read the x/y/z adc values
		// Now we'll calculate the accleration value into actual g's
		ax = (float) MPU9250Data[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
		ay = (float) MPU9250Data[1] * aRes - accelBias[1];
		az = (float) MPU9250Data[2] * aRes - accelBias[2];

		//   readGyroData(gyroCount);  // Read the x/y/z adc values

		// Calculate the gyro value into actual degrees per second
		gx = (float) MPU9250Data[4] * gRes; // get actual gyro value, this depends on scale being set
		gy = (float) MPU9250Data[5] * gRes;
		gz = (float) MPU9250Data[6] * gRes;
		imu.readMagData(magCount); // Read the x/y/z adc values
		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx = (float) magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
		my = (float) magCount[1] * mRes * magCalibration[1] - magBias[1];
		mz = (float) magCount[2] * mRes * magCalibration[2] - magBias[2];
		mx *= magScale[0];
		my *= magScale[1];
		mz *= magScale[2];
		currenttime = imuTimer->readTime();
		deltat = currenttime - lasttime; // set integration time in seconds by time elapsed since last filter update
		lasttime = currenttime;

		sum += deltat; // sum for averaging filter update rate
		++sumCount;

		// Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
		// the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
		// We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
		// For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
		// we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
		// positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
		// function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
		// This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
		// Pass gyro rate as rad/s
		//Profiler::tic(8);
		Madge::MadgwickQuaternionUpdate(-ax, ay, az, gx * M_PI / 180.0f,
				-gy * M_PI / 180.0f, -gz * M_PI / 180.0f, my, -mx, mz);
		//Profiler::toc(8);
		//  if(passThru)MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
		// Serial print and/or display at 0.5 s rate independent of data rates
		if (sum > .5) { // update LCD once per half-second independent of read rate
			if (SerialDebug) {
				printf("\ngRes = %f", gRes); printf("\nax = "); printf("%f", (int) 1000 * ax); printf(" ay = ");
				printf("%f", (int) 1000 * ay); printf(" az = "); printf("%f", (int) 1000 * az); printf(" mg");
				printf("\ngx = "); printf("%f", gx); printf(" gy = "); printf("%f", gy);
				printf(" gz = "); printf("%f", gz); printf(" deg/s");
				printf("\nmx = "); printf("%i", (int) mx); printf(" my = "); printf("%i", (int) my);
				printf(" mz = "); printf("%i", (int) mz); printf(" mG");
				printf("\nq0 = ");printf("%f", q[0]);printf(" qx = ");printf("%f", q[1]);printf(" qy = ");
				printf("%f", q[2]);printf(" qz = ");printf("%f", q[3]);
			}
			//tempCount = imu.readTempData();  // Read the gyro adc values
			//temperature = ((float) tempCount) / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
			// Print temperature in degrees Centigrade
			//printf("Gyro temperature is ");  printf(temperature, 1);  printf(" degrees C"); // Print T values to tenths of s degree C

			// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
			// In this coordinate system, the positive z-axis is down toward Earth.
			// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
			// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
			// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
			// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
			// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
			// applied in the correct order which for this configuration is yaw, pitch, and then roll.
			// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
			//Software AHRS:
			a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
			a22 = 1 - (2.0f * (pow(q[2], 2) + pow(q[3], 2)));
			a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
			a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
			a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
			pitch = -asinf(a32);
			roll = atan2f(a31, a33);
			yaw = atan2f(a12, a22);
			pitch *= 180.0f / M_PI;
			yaw *= 180.0f / M_PI;

			//Heading is defined for the Cartesian coordiate system, where 0 degrees points along the x-axis
			//Heading increases in the counterclockwise direction
			heading = degreeWrap(-yaw + 180 - MAG_DECL);
			roll *= 180.0f / M_PI;
			lin_ax = ax + a31;
			lin_ay = ay + a32;
			lin_az = az - a33;
			if (SerialDebug) {
				printf("\nYaw, Pitch, Roll: %f, %f, %f", yaw, pitch, roll);
				printf("\nHeading: %f", heading);
				printf("\nGrav_x, Grav_y, Grav_z: %f, %f, %f mg", -a31 * 1000,
						-a32 * 1000, a33 * 1000);
				printf("\nLin_ax, Lin_ay, Lin_az: %f, %f, %f mg", lin_ax * 1000,
						lin_ay * 1000, lin_az * 1000);
				printf("\nrate = ");
				printf("%f", (double) sumCount / sum);
				printf(" Hz");
			}

			// With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
			// >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
			// The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
			// the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
			// an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
			// filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
			// This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
			// This filter update rate should be fast enough to maintain accurate platform orientation for
			// stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
			// produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
			// The 3.3 V 8 MHz Pro Mini is doing pretty well!
			sumCount = 0; //incremented each filter update, reset here every half-second or so
			sum = 0; //sum of time intervals between filter updates, reset here every half-second or so
		}
	}
}

void unclodI2C() {
	iprintf("Attempting to unclod I2C bus\r\n");
	Pins[27].function(PIN_27_GPIO);//I2C for IMU
	Pins[29].function(PIN_29_GPIO);//I2C For IMU
	Pins[29].hiz();
	for(int i=0; i<256; i++) {
		Pins[27]=1;
		for (volatile int d=0; d<100; d++) asm(" nop");
		Pins[27]=0;
		for (volatile int d=0; d<100; d++) asm(" nop");
	}
	Pins[27].function(PIN_27_I2C0_SCL    );//I2C for IMU
	Pins[29].function(PIN_29_I2C0_SDA    );//I2C For IMU
	I2CResetPeripheral();
	OSTimeDly(5);
	//val = readReg((unsigned char *)0x75);  // Read WHO_AM_I register for MPU-9250
	uint8_t val = imu.readByte(0x68,0x75);
	iprintf("Retry ID=%02X\r\n",(int)val);
}

int whoAmICheck() { //Returns 0 for successful initialization, -1 for fail
	uint8_t val = imu.readByte(0x68,0x75);
	if (val != 0x71) { // WHO_AM_I should always be 0x71
		printf("Whoami: 0x%x, should be 0x71\n", val);
		unclodI2C();
	}
	if (val == 0x71) {
		printf("IMU I2C connection successful\n");
		return 0;
	}
	else {
		printf("IMU I2C connection failed\n");
		return -1;
	}
}


void IMURun() {
	//Create task stack in SRAM for performance benefits
	//Also put main task and idle task in SRAM by uncommenting
	//FAST_IDLE_TASK and FAST_MAIN_TASK in <constants.h> for
	//further performance benefits.
	//See "Intro to Profiling and Performance Optimization"
	//article on netburner.com for more info.
    OSSimpleTaskCreatewNameSRAM(IMUSampleLoop,IMU_TASK_PRIO,"IMU");
}

float getHeading() {
	return degreeWrap(heading-zeroOffset);
}

float degreeWrap(float deg) {
	while (deg>180)
		deg-=360;
	while (deg<-180)
		deg+=360;
	return deg;
}

void zeroHeading() {
	zeroOffset = heading;
}

float* getQuaternion() {
	return q;
}
