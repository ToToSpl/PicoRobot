#include "hardware/pio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "quadrature_encoder.pio.h"

#define MOTOR_LEFT_PIN_A 16
#define MOTOR_LEFT_PIN_B 17
#define MOTOR_LEFT_ENCODER_1 18
#define MOTOR_LEFT_ENCODER_2 19

#define STEPS_PER_ENCODER_ROT 16
#define ENCODER_ROT_PER_WHEEL_ROT 58
#define STEPS_PER_WHEEL_ROT (STEPS_PER_ENCODER_ROT * ENCODER_ROT_PER_WHEEL_ROT)
#define WHEEL_CIRCUM_M 0.2120575f
#define DISTANCE_PER_STEP_M (WHEEL_CIRCUM_M / (float)STEPS_PER_WHEEL_ROT)


int main() {

  stdio_init_all();

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  int new_value, delta, old_value = 0;
  bool led_state = false;

  PIO pio = pio0;
  const uint sm = 0;

  uint offset = pio_add_program(pio, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio, sm, offset, MOTOR_LEFT_ENCODER_1, 0);
  float distance = 0.0, speed = 0.0, delta_m;

  while (1) {
    // note: thanks to two's complement arithmetic delta will always
    // be correct even when new_value wraps around MAXINT / MININT
    new_value = quadrature_encoder_get_count(pio, sm);
    delta = new_value - old_value;
    old_value = new_value;

    delta_m = (float)delta * DISTANCE_PER_STEP_M;
    distance += delta_m;
    speed = delta_m * 10;

    printf("position %7.4f m, speed %7.4f m/s\n", distance, speed);

    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
    led_state = !led_state;
    
    sleep_ms(100);
  }

  return 0;
}
