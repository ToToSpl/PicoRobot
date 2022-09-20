#include "optical_sensor.h"
#include "hardware/gpio.h"

void optical_sensor_init(struct OpticalSensor *optical, uint pin,
                         float threshold, bool reversed) {
  adc_gpio_init(pin);
  uint offset = pin - GPIO_ADC_FIRST;

  optical->pin = pin;
  optical->offset = offset;
  optical->threshold = threshold;
  optical->reversed = reversed;
}

bool optical_sensor_measure(struct OpticalSensor *optical) {
  static uint current_offset = 0;
  static const float conversion_factor = 3.3f / (1 << 12);

  if (current_offset != optical->offset) {
    current_offset = optical->offset;
    adc_select_input(current_offset);
  }

  float measure = adc_read() * conversion_factor;
  bool trigger = measure > optical->threshold;

  return trigger ^ optical->reversed;
}

void bumper_sensor_init(struct BumperSensor *bumper, uint pin, bool reversed) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_set_pulls(pin, false, true);

  bumper->pin = pin;
  bumper->reversed = !reversed;
}

bool bumper_sensor_measure(struct BumperSensor *bumper) {
  bool measure = gpio_get(bumper->pin);
  return measure ^ bumper->reversed;
}
