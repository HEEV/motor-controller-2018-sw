#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include <settings_structs.h>

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

  ~SettingsManager() = default;

  SettingsManager(const SettingsManager &cpy) = delete;
  SettingsManager operator=(const SettingsManager &rhs) = delete;
};


#endif //SETTINGS_MANAGER_H