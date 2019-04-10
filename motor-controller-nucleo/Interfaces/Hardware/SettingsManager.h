#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include <settings_structs.h>
#include <TMC4671Interface.h>
#include <cstdio>
#include <Actions.h>

#ifdef __cplusplus
extern "C" {
  #include "tiny_printf.h"
}
#else
  #include "tiny_printf.h"
#endif

#define my_sprintf sprintf

extern TMC4671Interface* htmc4671;

class SettingsManager
{
private:
  // the settings that get saved into flash memory
  static MotorControllerValues_t flash_settings __attribute__((section(".mc_settings"))); 
  MotorControllerValues_t *user_settings;

  void load_settings() const;
  bool stale_settings() const;

public:
  static bool flash_write;

  SettingsManager(MotorControllerValues_t* values);
  void save_settings();

  void write_setting(MotorControllerParameter_t param, std::int32_t value);
  const char* get_setting_as_string(char* buff, MotorControllerParameter_t param);

  ~SettingsManager() = default;

  SettingsManager(const SettingsManager &cpy) = delete;
  SettingsManager operator=(const SettingsManager &rhs) = delete;
};


#endif //SETTINGS_MANAGER_H