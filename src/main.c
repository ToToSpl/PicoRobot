#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "encoder.h"
#include "motor_controller.h"

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

  struct MotorController motor_left;
  motor_contr_init(&motor_left, &encoder, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B,
                   1);
  motor_contr_set_spd(&motor_left, 0.f);

  uint8_t cnt = 0;
  int8_t dir = 1;
  float spd = 0.05f;

  while (1) {
    encoder_update(&encoder);

    if (cnt == 0 || cnt == 128) {
      spd = -spd;
      dir = -dir;
      motor_contr_set_spd(&motor_left, spd);
    }
    cnt += dir;

    motor_contr_update(&motor_left);

    printf("position %7.4f m, speed %7.4f m/s, err %7.4f\n", encoder.distance,
           encoder.speed, motor_left.err_prev);

    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
    led_state = !led_state;

    sleep_ms(20);
  }

  return 0;
}
