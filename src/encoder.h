#ifndef ENCODER_LIB_H
#define ENCODER_LIB_H

#include "hardware/pio.h"

#define ENCODER_MAX_RES 0.00023f

struct Encoder {
  PIO pio;
  uint sm;
  int taken_steps;
  float distance;
  float speed;
  float dir;
  uint64_t last_updated_us;
};

uint encoder_create_pio_program(PIO pio);

// second encoder must be at the next gpio port
void encoder_init(struct Encoder *encoder, PIO pio, uint sm,
                  uint first_sens_pin, bool direction, uint prog_offset);
void encoder_update(struct Encoder *encoder);

// shorter wait for data fetching
void encoder_update_two(struct Encoder *enc1, struct Encoder *enc2);

#endif
