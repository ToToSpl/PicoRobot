#include <stdio.h>
#include "pico/stdlib.h"

#define MOTOR_LEFT_PIN_A 16
#define MOTOR_LEFT_PIN_B 17
#define MOTOR_LEFT_ENCODER_1 18
#define MOTOR_LEFT_ENCODER_2 19

int main() {

  stdio_init_all();

  gpio_init(MOTOR_LEFT_ENCODER_1);
  gpio_set_dir(MOTOR_LEFT_ENCODER_1, GPIO_IN);
  gpio_pull_down(MOTOR_LEFT_ENCODER_1);

  gpio_init(MOTOR_LEFT_ENCODER_2);
  gpio_set_dir(MOTOR_LEFT_ENCODER_2, GPIO_IN);
  gpio_pull_down(MOTOR_LEFT_ENCODER_2);

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  bool encoder1, encoder2;
  bool led_state = false;
  while (true) {

    encoder1 = gpio_get(MOTOR_LEFT_ENCODER_1);
    encoder2 = gpio_get(MOTOR_LEFT_ENCODER_2);

    printf("%d\t%d\n", encoder1, encoder2);

    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
    led_state = !led_state;

    sleep_ms(500);
  }

  return 0;
}
