// hw_encoders.h — quadrature wheel encoders, decoded in a GPIO interrupt.
// You built this on Days 8–9 — this is the same idea, hardened.
#ifndef HW_ENCODERS_H
#define HW_ENCODERS_H

#include <stdint.h>

void hw_encoders_init(void);

// Signed counts since reset. ~358.3 counts per wheel revolution,
// 3.56 counts per mm (see ENC_COUNTS_PER_MM_X100 in tuning.h).
// Forward should count UP on both wheels — verify on the DIAG screen and
// fix with ENC_SIGN_LEFT/RIGHT in tuning.h if not.
int32_t hw_encoder_left(void);
int32_t hw_encoder_right(void);

void hw_encoders_reset(void);

#endif
