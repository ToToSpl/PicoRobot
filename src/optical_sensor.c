#include "optical_sensor.h"

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
