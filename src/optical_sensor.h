#ifndef OPTICAL_SENSOR_LIB
#define OPTICAL_SENSOR_LIB

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define BUMPER_SENS_THRESH 1.5f
#define FLOOR_SENS_THRESH 2.0f

#define GPIO_ADC_FIRST 26
#define GPIO_ADC_SECOND 27
#define GPIO_ADC_THIRD 28

struct OpticalSensor {
  uint pin;
  uint offset;
  bool reversed;
  float threshold;
};

void optical_sensor_init(struct OpticalSensor *optical, uint pin,
                         float threshold, bool reversed);

bool optical_sensor_measure(struct OpticalSensor *optical);

#endif
