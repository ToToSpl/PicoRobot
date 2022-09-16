#include <stdio.h>
#include "pico/stdlib.h"

#include "encoder.h"

#define MOTOR_LEFT_PIN_A 16
#define MOTOR_LEFT_PIN_B 17
#define MOTOR_LEFT_ENCODER_1 18
#define MOTOR_LEFT_ENCODER_2 19


int main() {

  stdio_init_all();

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  bool led_state = 1;

  struct Encoder encoder;

  uint pio_offset = encoder_create_pio_program(pio0);
  encoder_init(&encoder, pio0, 0, MOTOR_LEFT_ENCODER_1, 1, pio_offset);

  while (1) {

    encoder_update(&encoder);

    printf("position %7.4f m, speed %9.7f m/s\n", encoder.distance, encoder.speed);

    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
    led_state = !led_state;
    
    sleep_ms(100);
  }

  return 0;
}
