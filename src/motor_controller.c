#include "motor_controller.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#define PWM_WRAP UINT8_MAX
#define PWM_CLK_DIV 128.f

#define PID_K 800.0f
#define PID_P 1.0f
#define PID_I 8.0f
#define PID_D 0.005f

#define MIN_POWER 10.f

void motor_ctrl_init(MotorController *motor_contr, Encoder *encoder, uint pin_a,
                     uint pin_b, bool direction) {
  gpio_set_function(pin_a, GPIO_FUNC_PWM);
  gpio_set_function(pin_b, GPIO_FUNC_PWM);

  uint slice_num = pwm_gpio_to_slice_num(pin_a);
  pwm_set_wrap(slice_num, PWM_WRAP);
  pwm_set_clkdiv(slice_num, PWM_CLK_DIV);
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
  pwm_set_enabled(slice_num, true);

  motor_contr->motor_a_pin = pin_a;
  motor_contr->motor_b_pin = pin_b;
  motor_contr->dir = direction ? 1.f : -1.f;
  motor_contr->enc = encoder;
  motor_contr->pwm_slice_num = slice_num;
  motor_contr->last_updated = time_us_64();
  motor_contr->curr_ab = 1;
  motor_contr->err_prev = 0.f;
  motor_contr->err_int = 0.f;
  motor_contr->target_spd = 0.f;

  pwm_set_gpio_level(motor_contr->motor_a_pin, 0);
  pwm_set_gpio_level(motor_contr->motor_b_pin, 0);
}

void motor_ctrl_set_spd(MotorController *motor_contr, float target_spd) {
  motor_contr->target_spd = motor_contr->dir * target_spd;
}

void motor_ctrl_update(MotorController *motor_contr) {
  uint64_t now = time_us_64();
  float dt = 0.000001f * (float)(now - motor_contr->last_updated);
  motor_contr->last_updated = now;

  float err = motor_contr->target_spd - motor_contr->enc->speed;
  err = 0.7f * err + 0.3f * motor_contr->err_prev;

  float d = (err - motor_contr->err_prev) / dt;
  motor_contr->err_prev = err;

  motor_contr->err_int += err * dt;
  float i = motor_contr->err_int;

  float out = PID_K * (PID_P * err + PID_I * i + PID_D * d);

  if (out > -MIN_POWER && out < MIN_POWER)
    out = 0.f;

  if (out > 255.f)
    out = 255.f;
  else if (out < -255.f)
    out = -255.f;

  out *= motor_contr->dir;

  if (out >= 0.f) {
    if (motor_contr->curr_ab == 0) {
      motor_contr->curr_ab = 1;
      pwm_set_gpio_level(motor_contr->motor_b_pin, 0);
    }

    pwm_set_gpio_level(motor_contr->motor_a_pin, (uint16_t)out);

  } else {
    if (motor_contr->curr_ab == 1) {
      motor_contr->curr_ab = 0;
      pwm_set_gpio_level(motor_contr->motor_a_pin, 0);
    }

    pwm_set_gpio_level(motor_contr->motor_b_pin, (uint16_t)(-out));
  }
}
