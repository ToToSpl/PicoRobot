#ifndef OPTICAL_SENSOR_LIB
#define OPTICAL_SENSOR_LIB

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

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

struct BumperSensor {
  uint pin;
  bool reversed;
};

void bumper_sensor_init(struct BumperSensor *bumper, uint pin, bool reversed);

bool bumper_sensor_measure(struct BumperSensor *bumper);

#endif
