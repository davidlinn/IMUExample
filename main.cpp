#include <predef.h>
#include <stdio.h>
#include <ctype.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <smarttrap.h>
#include <taskmon.h>
#include <NetworkDebug.h>
#include <pins.h>
#include <pin_irq.h>
#include <multichanneli2c.h>
#include <constants.h>
#include <HiResTimer.h>
#include "IMUSetupAndSample.h"
#include "MPU9250.h"
#include "Boot_Settings.h"

#define IMU_TIMER 3

const char *AppName = "IMU Example";

MPU9250 imu; //Default constructor does nothing
NV_SettingsStruct NV_Settings; //Saved flash memory

extern "C" void UserMain(void *pd)
{
    //Basic network initialization
	InitializeStack();
    if (EthernetIP.IsNull()) GetDHCPAddress();
    iprintf("IP Address: ");
    ShowIP(EthernetIP);
    iprintf("\r\n");
    OSChangePrio(MAIN_PRIO);
    EnableAutoUpdate();
    #ifndef _DEBUG
    EnableSmartTraps();
    #endif
    #ifdef _DEBUG
    InitializeNetworkGDB_and_Wait();
    #endif


	//I2C (communication used for IMU)
	//  As written, I2C initialization will often fail unless the IMU is fully
	//  power cycled. In the case of the autonomous racecar (more info on netburner.com),
	//  I will often need to unplug the battery and plug it back in.
	MultiChannel_I2CInit();
	Pins[27].function(PIN_27_I2C0_SCL); //I2C Pins: change accordingly
	Pins[29].function(PIN_29_I2C0_SDA);

	//Re-construct MPU9250 object
	//passing in -1 assigns the next free HiResTimer to the imu
	imu = MPU9250(IMU_TIMER); //ignore warning that 'imu' set and not used

	//Initialize and calibrate IMU
	IMUSetup();

	//Create RTOS task that updates the IMU at a specified update rate
	IMURun();

    while (1)
    {
        OSTimeDly(TICKS_PER_SECOND);
    }
}
