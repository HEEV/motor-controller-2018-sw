#include "SettingsManager.h"
#include "flash.h"

#include <cstring>

bool SettingsManager::flash_write = false;

SettingsManager::SettingsManager(MotorControllerValues_t* user_values)
{
  // save the provided pointer
  user_settings = user_values;

  //check if the flash memory is in a useable state
  if(stale_settings()){
    save_settings();
  }
  else
  {
    load_settings();
  }
}

void SettingsManager::load_settings() const
{
  // copy the values from flash into ram
  memcpy(user_settings, &flash_settings, sizeof(MotorControllerValues_t));
}

bool SettingsManager::stale_settings() const
{
  uint8_t STALE_VALUE = 0xFF;
  size_t struct_size = sizeof(MotorControllerValues_t);
  uint8_t* ptr = (uint8_t*) &flash_settings;

  // walk over the memory as a 8-bit integer
  // comparing to the stale value
  for(size_t i = 0; i < struct_size; i++){
    if(*ptr != STALE_VALUE){
      return false;
    }
    ptr++;
  }
  return true;
}

void SettingsManager::save_settings()
{
  // make a temporary settings struct to zero unused values
  MotorControllerValues_t tmp_values = *user_settings;

  tmp_values.tmc4671.Setpoint = 0;

	//unlock the flash for writing
	flashUnlock();

	//erase flash
	flashErasePage((uint32_t) &flash_settings);

	// write the flash
	flashWriteMemBlock((uint32_t) &flash_settings, (uint8_t*) &tmp_values, sizeof(tmp_values));

	//relock the flash
	flashLock();

}