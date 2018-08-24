/*
 * Boot_Settings.h: This file defines the setting of user parameters at boot, used in AutoVehicle18
 * primarily for saving magnetometer calibration.
 *
 *  Created on: Jul 6, 2018
 *      Author: NetBurner
 */

#ifndef BOOT_SETTINGS_H_
#define BOOT_SETTINGS_H_

#include <basictypes.h>
#include <stdio.h>
#include <system.h>

// This key can be updated if you modify your structure. For example,
// you may need to do something special if you load a new version
// of s/w in a device that has the old structure values.
#define VERIFY_KEY (0x48666052)  // NV Settings key code
// This is the structure that will be stored in Flash. It is also used to update variables at runtime
// *** WARNING: CHANGE THE VERIFY KEY ANY TIME YOU MAKE A CHANGE TO THIS STRUCTURE ***
struct NV_SettingsStruct
{
      DWORD VerifyKey;  // Changes when the structure is modified so we can detect the change
      DWORD StructSize; // Store structure size
      float magBias[3];
      float magScale[3];
};
extern NV_SettingsStruct NV_Settings;

/*-------------------------------------------------------------------
 * Assign default values if VerifyKey has changed or is not initialized
 *------------------------------------------------------------------*/
inline void CheckNVSettings()
{
   iprintf("Checking NV_Settings User Parameter Flash...\r\n");
   iprintf( "Size of NV Structure: %ld Bytes\r\n", sizeof(NV_Settings) );

   NV_SettingsStruct *pData = ( NV_SettingsStruct * ) GetUserParameters();
   iprintf("Verify key = 0x%lX\r\n", pData->VerifyKey);

   // We will check the struct size as well to try and protect those who forgot to change
   // the verify key in order to avoid a crash or incorrect results
   if ( (pData->VerifyKey != VERIFY_KEY ) || (pData->StructSize != sizeof(NV_Settings)) )
   {
      if (pData->VerifyKey != VERIFY_KEY ) iprintf("Flash verification key has changed - initializing Flash\r\n");
      if (pData->StructSize != sizeof(NV_Settings)) iprintf("Struct size has changed - initializing Flash\r\n");
      NV_Settings.VerifyKey = VERIFY_KEY;
      NV_Settings.StructSize = sizeof(NV_Settings);

      for (int i = 0; i < 3; i++)
          NV_Settings.magBias[i] = 0;
      for (int i = 0; i < 3; i++)
          NV_Settings.magScale[i] = 0;

      SaveUserParameters( &NV_Settings, sizeof( NV_Settings ) );
   }
   else
   {
      iprintf("Flash verification is VALID - reading values from Flash\r\n");
      NV_Settings.VerifyKey = pData->VerifyKey;
      NV_Settings.StructSize = pData->StructSize;
      for (int i = 0; i < 3; i++)
          NV_Settings.magBias[i] = pData->magBias[i];
      for (int i = 0; i < 3; i++)
          NV_Settings.magScale[i] = pData->magScale[i];
   }
}



/*-------------------------------------------------------------------
 Display the values of the NV_Settings structure.
 ------------------------------------------------------------------*/
inline void DisplayNVSettings()
{
   for (int i = 0; i < 3; i++)
      iprintf("%f ", NV_Settings.magBias[i]);
   for (int i = 0; i < 3; i++)
      iprintf("%f ", NV_Settings.magScale[i]);
   iprintf("\r\n");
}

#endif /* BOOT_SETTINGS_H_ */
