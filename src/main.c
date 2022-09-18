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

  struct Encoder encoder;

  uint pio_offset = encoder_create_pio_program(pio0);
  encoder_init(&encoder, pio0, 0, MOTOR_LEFT_ENCODER_1, 0, pio_offset);

  struct MotorController motor_left;
  motor_contr_init(&motor_left, &encoder, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B,
                   1); // direction reversed due to wiring

  uint16_t cnt = 0;
  float dir = 0.01f;
  float spd = 0.05f;

  while (1) {
    encoder_update(&encoder);

/*
    if (cnt >= 100) {
      cnt = 0;
      if (spd < -0.05f || spd > 0.05f) {
        dir = -dir;
      }
      spd += dir;
      motor_contr_set_spd(&motor_left, spd);
    }
  */
    if(cnt == 200) {
      cnt = 0;
      spd = -spd;
      motor_contr_set_spd(&motor_left, spd);
    }
    cnt++;

    motor_contr_update(&motor_left);

    printf("%.8f %.8f %.8f\n", spd, encoder.speed, motor_left.err_prev);

    sleep_ms(10);
  }

  return 0;
}
