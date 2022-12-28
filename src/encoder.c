#include "encoder.h"

#include "hardware/pio.h"
#include "hardware/timer.h"

#include "quadrature_encoder.pio.h"

// encoder params
#define STEPS_PER_ENCODER_ROT 16
#define ENCODER_ROT_PER_WHEEL_ROT 58
#define STEPS_PER_WHEEL_ROT (STEPS_PER_ENCODER_ROT * ENCODER_ROT_PER_WHEEL_ROT)
#define WHEEL_CIRCUM_M 0.2120575f
#define DISTANCE_PER_STEP_M (WHEEL_CIRCUM_M / (float)STEPS_PER_WHEEL_ROT)

uint encoder_create_pio_program(PIO pio) {
  uint offset = pio_add_program(pio, &quadrature_encoder_program);
  return offset;
}

void encoder_init(Encoder *encoder, PIO pio, uint sm, uint first_sens_pin,
                  bool direction, uint prog_offset) {

  quadrature_encoder_program_init(pio, sm, prog_offset, first_sens_pin, 0);

  encoder->pio = pio;
  encoder->sm = sm;
  encoder->distance = 0.0f;
  encoder->speed = 0.0f;
  encoder->dir = direction ? 1.0f : -1.0f;
  encoder->taken_steps = 0;
  encoder->last_updated_us = time_us_64();
}

inline void encoder_update_math(Encoder *encoder, int32_t fetch_count) {
  int32_t delta = fetch_count - encoder->taken_steps;
  float delta_m = encoder->dir * (float)delta * DISTANCE_PER_STEP_M;

  encoder->taken_steps = fetch_count;
  encoder->distance += delta_m;

  uint64_t now = time_us_64();
  float dt = 0.000001f * (float)(now - encoder->last_updated_us);
  encoder->last_updated_us = now;

  encoder->speed = delta_m / dt;
}

void encoder_update(Encoder *encoder) {
  int32_t fetch_count = quadrature_encoder_get_count(encoder->pio, encoder->sm);
  encoder_update_math(encoder, fetch_count);
}

void encoder_update_two(struct Encoder *enc1, struct Encoder *enc2) {
  quadrature_encoder_request_count(enc1->pio, enc1->sm);
  quadrature_encoder_request_count(enc2->pio, enc2->sm);

  int32_t cnt1 = quadrature_encoder_fetch_count(enc1->pio, enc1->sm);
  int32_t cnt2 = quadrature_encoder_fetch_count(enc2->pio, enc2->sm);

  encoder_update_math(enc1, cnt1);
  encoder_update_math(enc2, cnt2);
}
