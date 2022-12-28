#ifndef OPTICAL_SENSOR_LIB
#define OPTICAL_SENSOR_LIB

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define FLOOR_SENS_THRESH 2.0f

#define GPIO_ADC_FIRST 26
#define GPIO_ADC_SECOND 27
#define GPIO_ADC_THIRD 28

typedef struct OpticalSensor {
  uint pin;
  uint offset;
  bool reversed;
  float threshold;
} OpticalSensor;

void optical_sensor_init(OpticalSensor *optical, uint pin, float threshold,
                         bool reversed);

bool optical_sensor_measure(OpticalSensor *optical);

typedef struct BumperSensor {
  uint pin;
  bool reversed;
} BumperSensor;

void bumper_sensor_init(BumperSensor *bumper, uint pin, bool reversed);

bool bumper_sensor_measure(BumperSensor *bumper);

#endif
