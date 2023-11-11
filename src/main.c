#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "encoder.h"
#include "motor_controller.h"
#include "optical_sensor.h"
#include "pinout.h"

BumperSensor bumperL, bumperR;
BumperSensor floorL1, floorL2, floorR1, floorR2;
Encoder encoderL, encoderR;
MotorController motor_left, motor_right;

void main_core1() {
  char chr;
  uint32_t ptr = 0;
  char buffer[64] = {0};
  float left_speed, right_speed;
  uint64_t last_updated = time_us_64();

  while (1) {
    printf("{"
           "\"BUMPER_L\": %d,"
           "\"BUMPER_R\": %d,"
           "\"FLOOR_L1\": %d,"
           "\"FLOOR_L2\": %d,"
           "\"FLOOR_R2\": %d,"
           "\"FLOOR_R1\": %d,"
           "\"ENCODER_L\": %f,"
           "\"ENCODER_R\": %f"
           "}\n",
           bumper_sensor_measure(&bumperL), bumper_sensor_measure(&bumperR),
           bumper_sensor_measure(&floorL1), bumper_sensor_measure(&floorL2),
           bumper_sensor_measure(&floorR2), bumper_sensor_measure(&floorR1),
           encoderL.distance, encoderR.distance);
    sleep_ms(30);

    chr = getchar_timeout_us(0);
    if (chr == 'R') {
      buffer[ptr++] = chr;
      while (chr != 'E' && ptr < 64) {
        chr = getchar_timeout_us(100);
        buffer[ptr++] = chr;
      }

      if (sscanf(buffer, "R %f L %f E", &left_speed, &right_speed) == 2) {
        last_updated = time_us_64();
        motor_ctrl_set_spd(&motor_left, left_speed);
        motor_ctrl_set_spd(&motor_right, right_speed);
      };

    } else {
      ptr = 0;

      if (time_us_64() - last_updated > 500000) {
        motor_ctrl_set_spd(&motor_left, 0.0f);
        motor_ctrl_set_spd(&motor_right, 0.0f);
      }
    }
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

  motor_ctrl_init(&motor_left, &encoderL, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B,
                  1);
  motor_ctrl_set_spd(&motor_left, 0.0f);

  motor_ctrl_init(&motor_right, &encoderR, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B,
                  1);
  motor_ctrl_set_spd(&motor_right, 0.0f);

  multicore_launch_core1(main_core1);

  while (1) {
    encoder_update(&encoderL);
    encoder_update(&encoderR);
    motor_ctrl_update(&motor_left);
    motor_ctrl_update(&motor_right);

    sleep_ms(10);
  }

  return 0;
}
