#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "encoder.h"
#include "motor_controller.h"
#include "optical_sensor.h"
#include "pinout.h"

BumperSensor bumperL, bumperR;
BumperSensor floorL1, floorL2, floorR1, floorR2;
Encoder encoderL, encoderR;

void main_core1() {
  while (1) {

    printf("{\
              BUMPER_L: %d,\
              BUMPER_R: %d,\
              FLOOR_L1: %d,\
              FLOOR_L2: %d,\
              FLOOR_R2: %d,\
              FLOOR_R1: %d,\
              ENCODER_L: %f,\
              ENCODER_R: %f\
            }\n",
           bumper_sensor_measure(&bumperL), bumper_sensor_measure(&bumperR),
           bumper_sensor_measure(&floorL1), bumper_sensor_measure(&floorL2),
           bumper_sensor_measure(&floorR2), bumper_sensor_measure(&floorR1),
           encoderL.distance, encoderR.distance);
    sleep_ms(10);
  }
}

int main() {

  stdio_init_all();

  // bumpers
  bumper_sensor_init(&bumperL, BUMPER_LEFT, 0);
  bumper_sensor_init(&bumperR, BUMPER_RIGHT, 0);
  // floor sensors (set as digital sensors)
  bumper_sensor_init(&floorL1, FLOOR_S_L1, 0);
  bumper_sensor_init(&floorL2, FLOOR_S_L2, 0);
  bumper_sensor_init(&floorR1, FLOOR_S_R1, 0);
  bumper_sensor_init(&floorR2, FLOOR_S_R2, 0);

  uint pio_offset = encoder_create_pio_program(pio0);
  encoder_init(&encoderL, pio0, 1, MOTOR_LEFT_ENCODER_1, 0, pio_offset);
  encoder_init(&encoderR, pio0, 0, MOTOR_RIGHT_ENCODER_1, 1, pio_offset);

  MotorController motor_left, motor_right;
  motor_ctrl_init(&motor_left, &encoderL, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B,
                  1);

  motor_ctrl_init(&motor_right, &encoderR, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B,
                  1);

  multicore_launch_core1(main_core1);

  uint16_t cnt = 0;
  float dir = 0.01f;
  float spd = 0.0f;
  motor_ctrl_set_spd(&motor_left, spd);
  motor_ctrl_set_spd(&motor_right, spd);

  while (1) {
    encoder_update(&encoderL);
    encoder_update(&encoderR);

    if (cnt >= 30) {
      cnt = 0;
      if (spd < -0.25f || spd > 0.25f) {
        dir = -dir;
      }
      spd += dir;
      motor_ctrl_set_spd(&motor_left, -spd);
      motor_ctrl_set_spd(&motor_right, spd);
    }
    cnt++;

    motor_ctrl_update(&motor_left);
    motor_ctrl_update(&motor_right);

    sleep_ms(10);
  }

  return 0;
}
