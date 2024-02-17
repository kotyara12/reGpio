/* 
   RU: Level switching tracking on GPIO interrupt with debounce
   EN: Отслеживание переключения уровней на GPIO через прерывание с подавлением дребезга контактов
   --------------------------------------------------------------------------------
   (с) 2022-2024 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_BUTTON_H__
#define __RE_BUTTON_H__

#include <stdint.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "rTypes.h"

#define CONFIG_BUTTON_DEBOUNCE_TIME_US 50000
#define CONFIG_PIR_DEBOUNCE_TIME_US    1000000
#define CONFIG_BUTTON_LONG_PRESS_MS    1000

#ifdef __cplusplus
extern "C" {
#endif

class reGPIO {
public:
  reGPIO(uint8_t gpio_num, uint8_t active_level, bool internal_pull, bool interrupt_enabled, uint32_t debounce_time, cb_gpio_change_t callback);
  ~reGPIO();

  void setEventGroup(EventGroupHandle_t event_group, const uint32_t bits_press, const uint32_t bits_long_press);
  void setCallback(cb_gpio_change_t callback);
  bool initGPIO();
  bool setInternalPull(bool enabled);
  bool activate(bool activate_pull);
  bool deactivate(bool deactivate_pull);
  bool read();
  uint8_t getState();
  
  void onInterrupt();
  void onDebounce();
private:
  gpio_num_t _gpio_num;
  uint8_t _active_level = 1;
  uint8_t _state = 0xFF;
  bool _internal_pull = false;
  bool _interrupt_enabled = true;
  bool _interrupt_set = false;
  uint32_t _debounce_time = 0;
  uint64_t _timestamp = 0;
  EventGroupHandle_t _event_group = nullptr;
  uint32_t _bits_press = 0x00; 
  uint32_t _bits_long_press = 0x00;
  cb_gpio_change_t _callback = nullptr;
  esp_timer_handle_t _timer = nullptr;

  bool readGPIO(bool isr);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_BUTTON_H__