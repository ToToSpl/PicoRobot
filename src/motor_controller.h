#ifndef MOTOR_CONTROLLER_LIB
#define MOTOR_CONTROLLER_LIB

#include "encoder.h"

struct MotorController {
  uint motor_a_pin;
  uint motor_b_pin;
  float dir;
  float target_spd;
  struct Encoder *enc;
  float err_int, err_prev;
  uint pwm_slice_num;
  uint64_t last_updated;
  bool curr_ab;
};

void motor_contr_init(struct MotorController *motor_contr,
                      struct Encoder *encoder, uint pin_a, uint pin_b,
                      bool direction);

void motor_contr_set_spd(struct MotorController *motor_contr, float target_spd);

void motor_contr_update(struct MotorController *motor_contr);

#endif
