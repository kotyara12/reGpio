#include "reGPIO.h"
#include "reEvents.h"
#include "reEsp32.h"
#include "rLog.h"
#include "string.h"

static const char* logTAG = "GPIO";

static void IRAM_ATTR gpioIsrHandler(void* arg)
{
  reGPIO* gpio = (reGPIO*)arg;
  gpio->onInterrupt();
}

static void debounceTimeout(void* arg) 
{
  reGPIO* gpio = (reGPIO*)arg;
  gpio->onDebounce();
}

reGPIO::reGPIO(uint8_t gpio_num, uint8_t active_level, bool internal_pull, bool interrupt_enabled, uint32_t debounce_time, cb_gpio_change_t callback)
{
  _gpio_num = (gpio_num_t)gpio_num;
  _active_level = active_level;
  _internal_pull = internal_pull;
  _debounce_time = debounce_time;
  _event_group = nullptr;
  _bits_press = 0x00;
  _bits_long_press = 0x00;
  _callback = callback;
  _timer = nullptr;
  _interrupt_enabled = interrupt_enabled;
  _interrupt_set = false;
  _state = 0xFF;
}

reGPIO::~reGPIO()
{
  if (_interrupt_enabled || _interrupt_set) {
    gpio_intr_disable(_gpio_num);
    gpio_isr_handler_remove(_gpio_num);
    _interrupt_set = false;
  };

  if (_timer) {
    if (esp_timer_is_active(_timer)) {
      esp_timer_stop(_timer);
    };
    esp_timer_delete(_timer);
    _timer = nullptr;
  };
}

void reGPIO::setEventGroup(EventGroupHandle_t event_group, const uint32_t bits_on, const uint32_t bits_off, const uint32_t bits_press, const uint32_t bits_long_press)
{
  _event_group = event_group;
  _bits_on = bits_on;
  _bits_off = bits_off;
  _bits_press = bits_press;
  _bits_long_press = bits_long_press;
}

void reGPIO::setCallback(cb_gpio_change_t callback)
{
  _callback = callback;
}

int8_t reGPIO::initGPIO()
{
  // Configure GPIO
  gpio_config_t cfgGPIO = {
      .pin_bit_mask = BIT64(_gpio_num),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = (_internal_pull && (_active_level == 0)) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
      .pull_down_en = (_internal_pull && (_active_level != 0)) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
      .intr_type = _interrupt_enabled ? GPIO_INTR_ANYEDGE : GPIO_INTR_DISABLE,
  };
  RE_OK_CHECK(gpio_config(&cfgGPIO), return -1);

  // Create debounce timer
  if ((_debounce_time > 0) && (_timer == nullptr)) {
    esp_timer_create_args_t tmr_cfg;
    tmr_cfg.arg = this;
    tmr_cfg.callback = debounceTimeout;
    tmr_cfg.dispatch_method = ESP_TIMER_TASK;
    tmr_cfg.name = "debounce";
    tmr_cfg.skip_unhandled_events = false;
    RE_OK_CHECK(esp_timer_create(&tmr_cfg, &_timer), return -2);
  };

  // Install interrupt handler
  if (_interrupt_enabled && !_interrupt_set) {
    RE_OK_CHECK(gpio_isr_handler_add(_gpio_num, gpioIsrHandler, this), return -3);
    RE_OK_CHECK(gpio_intr_enable(_gpio_num), return -4);
    _interrupt_set = true;
  };
  
  rlog_i(logTAG, "GPIO %d initialized", _gpio_num);

  // Read current state
  if (readGPIO(false)) {
    return 1;
  };
  return 0;
}

bool reGPIO::setInternalPull(bool enabled)
{
  if (enabled) {
    if (_active_level) {
      RE_OK_CHECK(gpio_set_pull_mode(_gpio_num, GPIO_PULLDOWN_ONLY), return false);
    } else {
      RE_OK_CHECK(gpio_set_pull_mode(_gpio_num, GPIO_PULLUP_ONLY), return false);
    };
  } else {
    RE_OK_CHECK(gpio_set_pull_mode(_gpio_num, GPIO_FLOATING), return false);
  };
  return true;
}

bool reGPIO::activate(bool activate_pull)
{
  if (activate_pull) {
    setInternalPull(_internal_pull);
  };
  if (_interrupt_enabled && !_interrupt_set) {
    RE_OK_CHECK(gpio_intr_enable(_gpio_num), return false);
    _interrupt_set = true;
    return readGPIO(false);
  };
  return true;
}

bool reGPIO::deactivate(bool deactivate_pull)
{
  if (deactivate_pull) {
    setInternalPull(false);
  };
  if (_interrupt_enabled && _interrupt_set) {
    RE_OK_CHECK(gpio_intr_disable(_gpio_num), return false);
    _interrupt_set = false;
  };
  return true;
}

bool reGPIO::readGPIO(bool isr)
{
  uint8_t newState;
  gpio_get_level(_gpio_num) == _active_level ? newState = 1 : newState = 0;
  // GPIO physical level has changed
  if (_state != newState) {
    // Prepare data for event (max 4 bytes)
    gpio_data_t evt_data;
    evt_data.bus = 0;     // Physical GPIO, no I2C
    evt_data.address = 0; // Physical GPIO, no I2C
    evt_data.pin = (uint8_t)_gpio_num;
    evt_data.value = newState;

    // Calculate duration
    uint32_t duration = 0;
    if (_timestamp > 0) {
      duration = (uint32_t)((esp_timer_get_time() / 1000ULL) - _timestamp);
    };
    _timestamp = esp_timer_get_time() / 1000ULL;

    // Send basic event
    if (isr) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      BaseType_t xResult = pdFAIL;
      // Select dispatch method 
      if (_event_group) {
        // Through an event group
        if (newState) {
          if (_bits_on != 0) {
            xResult = xEventGroupSetBitsFromISR(_event_group, _bits_on, &xHigherPriorityTaskWoken);
          };
        } else {
          if (_bits_off != 0) {
            xResult = xEventGroupSetBitsFromISR(_event_group, _bits_off, &xHigherPriorityTaskWoken);
          };
        };
        if ((_state == 1) && (newState == 0)) {
          if (duration < CONFIG_BUTTON_LONG_PRESS_MS) {
            if (_bits_press != 0) {
              xResult = xEventGroupSetBitsFromISR(_event_group, _bits_press, &xHigherPriorityTaskWoken);
            };
          } else {
            if (_bits_long_press != 0) {
              xResult = xEventGroupSetBitsFromISR(_event_group, _bits_long_press, &xHigherPriorityTaskWoken);
            };
          };
        };
      } else {
        // Through an event loop
        xResult = eventLoopPostFromISR(RE_GPIO_EVENTS, RE_GPIO_CHANGE, &evt_data, sizeof(evt_data), &xHigherPriorityTaskWoken);
        // If button was released, send additional event
        if ((_state == 1) && (newState == 0)) {
          if (duration < CONFIG_BUTTON_LONG_PRESS_MS) {
            xResult = eventLoopPostFromISR(RE_GPIO_EVENTS, RE_GPIO_BUTTON, &evt_data, sizeof(evt_data), &xHigherPriorityTaskWoken);
          } else {
            xResult = eventLoopPostFromISR(RE_GPIO_EVENTS, RE_GPIO_LONG_BUTTON, &evt_data, sizeof(evt_data), &xHigherPriorityTaskWoken);
          };
        };
      };
      // Callback
      if (_callback) {
        _callback((void*)this, evt_data, duration);
      };
      // Switch context
      if (xResult == pdTRUE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      };
    } else {
      // Select dispatch method 
      if (_event_group) {
        // Through an event group
        if (newState) {
          if (_bits_on != 0) {
            xEventGroupSetBits(_event_group, _bits_on);
          };
        } else {
          if (_bits_off != 0) {
            xEventGroupSetBits(_event_group, _bits_off);
          };
        };
        if ((_state == 1) && (newState == 0)) {
          if (duration < CONFIG_BUTTON_LONG_PRESS_MS) {
            if (_bits_press != 0) {
              xEventGroupSetBits(_event_group, _bits_press);
            };
          } else {
            if (_bits_long_press != 0) {
              xEventGroupSetBits(_event_group, _bits_long_press);
            };
          };
        };
      } else {
        // Through an event loop
        eventLoopPost(RE_GPIO_EVENTS, RE_GPIO_CHANGE, &evt_data, sizeof(evt_data), portMAX_DELAY);
        // If button was released, send additional event
        if ((_state == 1) && (newState == 0)) {
          if (duration < CONFIG_BUTTON_LONG_PRESS_MS) {
            eventLoopPost(RE_GPIO_EVENTS, RE_GPIO_BUTTON, &evt_data, sizeof(evt_data), portMAX_DELAY);
          } else {
            eventLoopPost(RE_GPIO_EVENTS, RE_GPIO_LONG_BUTTON, &evt_data, sizeof(evt_data), portMAX_DELAY);
          };
        };
      };
      // Callback
      if (_callback) {
        _callback((void*)this, evt_data, duration);
      };
      vPortYield();
    };
  };
  _state = newState;
  return newState;
}

bool reGPIO::read()
{
  return readGPIO(false);
}

uint8_t reGPIO::getState()
{
  return _state;
}

void reGPIO::onInterrupt()
{
  if (_timer) {
    if (esp_timer_is_active(_timer)) {
      esp_timer_stop(_timer);
    };
    if (esp_timer_start_once(_timer, _debounce_time) == ESP_OK) {
      if (_interrupt_enabled && (gpio_intr_disable(_gpio_num) == ESP_OK)) {
        _interrupt_set = false;
      };
    } else { 
      readGPIO(true); 
    };
  } else { 
    readGPIO(true); 
  };
}

void reGPIO::onDebounce()
{
  // Stop debounce timer, if active
  if (_timer) {
    if (esp_timer_is_active(_timer)) {
      esp_timer_stop(_timer);
    };
  };

  // Enable interrupts
  if (_interrupt_enabled) {
    _interrupt_set = gpio_intr_enable(_gpio_num) == ESP_OK;
  };

  // Read new status
  readGPIO(false);
}
