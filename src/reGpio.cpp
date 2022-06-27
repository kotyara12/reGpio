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

reGPIO::reGPIO(uint8_t gpio_num, uint8_t active_level, bool internal_pull, uint32_t debounce_time, cb_gpio_change_t callback)
{
  _gpio_num = (gpio_num_t)gpio_num;
  _active_level = active_level;
  _internal_pull = internal_pull;
  _debounce_time = debounce_time;
  _callback = callback;
  _timer = nullptr;
  _interrupt_set = false;
  _state = 0xFF;
}

reGPIO::~reGPIO()
{
  gpio_intr_disable(_gpio_num);
  gpio_isr_handler_remove(_gpio_num);
  _interrupt_set = false;

  if (_timer) {
    if (esp_timer_is_active(_timer)) {
      esp_timer_stop(_timer);
    };
    esp_timer_delete(_timer);
    _timer = nullptr;
  };
}

void reGPIO::setCallback(cb_gpio_change_t callback)
{
  _callback = callback;
}

bool reGPIO::initGPIO()
{
  esp_err_t err;
  // Set GPIO mode
  if (!_interrupt_set) {
    // rlog_d(logTAG, "Init GPIO %d...", _gpio_num);
    gpio_pad_select_gpio(_gpio_num);
    RE_OK_CHECK(gpio_set_direction(_gpio_num, GPIO_MODE_INPUT), return false);
    if (_internal_pull) {
      if (_active_level) {
        RE_OK_CHECK(gpio_set_pull_mode(_gpio_num, GPIO_PULLDOWN_ONLY), return false);
      } else {
        RE_OK_CHECK(gpio_set_pull_mode(_gpio_num, GPIO_PULLUP_ONLY), return false);
      };
    } else {
      RE_OK_CHECK(gpio_set_pull_mode(_gpio_num, GPIO_FLOATING), return false);
    };
  };

  // Create debounce timer
  if ((_debounce_time > 0) && !(_timer)) {
    // rlog_d(logTAG, "Init GPIO %d debounce timer...", _gpio_num);
    esp_timer_create_args_t tmr_cfg;
    tmr_cfg.arg = this;
    tmr_cfg.callback = debounceTimeout;
    tmr_cfg.dispatch_method = ESP_TIMER_TASK;
    tmr_cfg.name = "debounce";
    tmr_cfg.skip_unhandled_events = false;
    RE_OK_CHECK(esp_timer_create(&tmr_cfg, &_timer), return false);
  };

  // Install interrupt
  if (!_interrupt_set) {
    // rlog_d(logTAG, "Init GPIO %d interrupt...", _gpio_num);
    RE_OK_CHECK(gpio_set_intr_type(_gpio_num, GPIO_INTR_ANYEDGE), return false);
    RE_OK_CHECK(gpio_isr_handler_add(_gpio_num, gpioIsrHandler, this), return false);
    RE_OK_CHECK(gpio_intr_enable(_gpio_num), return false);
    _interrupt_set = true;
  };
  
  rlog_i(logTAG, "GPIO %d initialized", _gpio_num);

  // Read current state
  return readGPIO(false);
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
      BaseType_t xHigherPriorityTaskWoken, xResult;
      xResult = eventLoopPostFromISR(RE_GPIO_EVENTS, RE_GPIO_CHANGE, &evt_data, sizeof(evt_data), &xHigherPriorityTaskWoken);
      // If button was released, send additional event
      if ((_state == 1) && (newState == 0)) {
        if (duration < CONFIG_BUTTON_LONG_PRESS_MS) {
          xResult = eventLoopPostFromISR(RE_GPIO_EVENTS, RE_GPIO_BUTTON, &evt_data, sizeof(evt_data), &xHigherPriorityTaskWoken);
        } else {
          xResult = eventLoopPostFromISR(RE_GPIO_EVENTS, RE_GPIO_LONG_BUTTON, &evt_data, sizeof(evt_data), &xHigherPriorityTaskWoken);
        };
      };
      // Callback
      if (_callback) {
        _callback((void*)this, evt_data, duration);
      };
      // Switch context
      if (xResult == pdPASS) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      };
    } else {
      eventLoopPost(RE_GPIO_EVENTS, RE_GPIO_CHANGE, &evt_data, sizeof(evt_data), portMAX_DELAY);
      // If button was released, send additional event
      if ((_state == 1) && (newState == 0)) {
        if (duration < CONFIG_BUTTON_LONG_PRESS_MS) {
          eventLoopPost(RE_GPIO_EVENTS, RE_GPIO_BUTTON, &evt_data, sizeof(evt_data), portMAX_DELAY);
        } else {
          eventLoopPost(RE_GPIO_EVENTS, RE_GPIO_LONG_BUTTON, &evt_data, sizeof(evt_data), portMAX_DELAY);
        };
      };
      // Callback
      if (_callback) {
        _callback((void*)this, evt_data, duration);
      };
    };
  };
  _state = newState;
  return newState;
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
      if (gpio_intr_disable(_gpio_num) == ESP_OK) {
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
  // Read new status
  readGPIO(false);

  // Stop debounce timer, if active
  if (_timer) {
    if (esp_timer_is_active(_timer)) {
      esp_timer_stop(_timer);
    };
  };

  // Enable interrupts
  _interrupt_set = gpio_intr_enable(_gpio_num) == ESP_OK;
}
