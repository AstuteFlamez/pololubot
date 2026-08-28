// hw_encoders.h — quadrature wheel encoders, decoded in a GPIO interrupt.
#ifndef HW_ENCODERS_H
#define HW_ENCODERS_H

#include <stdint.h>

// Configure GP8/GP9 (right) and GP12/GP13 (left) as inputs, seed the decoder
// from their current levels, and enable both-edge interrupts on all four.
// This installs the RP2040's single shared GPIO callback via
// gpio_set_irq_enabled_with_callback(), so any other module wanting GPIO
// interrupts must cooperate with it rather than register its own. Does not
// zero the counters — call hw_encoders_reset() for that.
void hw_encoders_init(void);

// Signed count accumulated since the last reset, already multiplied by
// ENC_SIGN_LEFT / ENC_SIGN_RIGHT (tuning.h) so that forward motion counts UP
// on both wheels. Scale: ~358.3 counts per wheel revolution, 3.56 counts per
// mm (ENC_COUNTS_PER_MM_X100). Cheap and interrupt-safe — one 32-bit load.
// If forward makes a wheel count down, flip that wheel's sign in tuning.h;
// the DIAG screen shows both counts live.
int32_t hw_encoder_left(void);
int32_t hw_encoder_right(void);

// Zero both counters. The two are zeroed by separate stores, so an edge that
// lands between them is credited to one wheel and not the other; that is one
// count, and no caller cares.
void hw_encoders_reset(void);

#endif
