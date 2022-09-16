#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "encoder.h"

#define MOTOR_LEFT_PIN_A 16
#define MOTOR_LEFT_PIN_B 17
#define MOTOR_LEFT_ENCODER_1 18
#define MOTOR_LEFT_ENCODER_2 19

int main() {

  stdio_init_all();

  // gpio_init(PICO_DEFAULT_LED_PIN);
  // gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  // bool led_state = 1;
  gpio_set_function(MOTOR_LEFT_PIN_A, GPIO_FUNC_PWM);
  gpio_set_function(MOTOR_LEFT_PIN_B, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(MOTOR_LEFT_PIN_A);
  pwm_set_wrap(slice_num, UINT8_MAX);
  pwm_set_clkdiv(slice_num, 128.f);
  uint16_t led_bright = 0;
  int16_t dir = 1;
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
  pwm_set_enabled(slice_num, true);

  struct Encoder encoder;

  uint pio_offset = encoder_create_pio_program(pio0);
  encoder_init(&encoder, pio0, 0, MOTOR_LEFT_ENCODER_1, 1, pio_offset);

  while (1) {

    led_bright = 1;
    dir = 1;
    while (led_bright != 0) {
      if (led_bright == UINT8_MAX)
        dir = -1;
      led_bright += dir;
      pwm_set_gpio_level(MOTOR_LEFT_PIN_A, led_bright);
      sleep_ms(20);
    }
    //disable PIN_A
    pwm_set_gpio_level(MOTOR_LEFT_PIN_A, 0);

    led_bright = 1;
    dir = 1;
    while (led_bright != 0) {
      if (led_bright == UINT8_MAX)
        dir = -1;
      led_bright += dir;
      pwm_set_gpio_level(MOTOR_LEFT_PIN_B, led_bright);
      sleep_ms(20);
    }
    //disable PIN_B
    pwm_set_gpio_level(MOTOR_LEFT_PIN_B, 0);




    // encoder_update(&encoder);

    // printf("position %7.4f m, speed %9.7f m/s\n", encoder.distance,
    // encoder.speed);

    // gpio_put(PICO_DEFAULT_LED_PIN, led_state);
    // led_state = !led_state;

    sleep_ms(20);
  }

  return 0;
}
