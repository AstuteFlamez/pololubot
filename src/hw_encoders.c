// hw_encoders.c — 4× quadrature decode in a GPIO IRQ.
//
// Right encoder: GP8 (A) / GP9 (B).  Left: GP12 (A) / GP13 (B).
// Every edge on any of the four pins fires one interrupt; the handler
// reads both pins of each channel and walks a 16-entry transition table:
// index = (previous AB << 2) | current AB. Legal gray-code steps give ±1,
// "impossible" jumps (missed edge, bounce) give 0 instead of corrupting
// the count. Counting all 4 edges per cycle is what gets us 358.3
// counts/rev from a 12-count magnetic disc through the ~29.86:1 gearbox.
//
// Why an IRQ and not polling? At speed-run pace the wheels produce edges
// every ~100 µs while the main loop is busy doing ~1.1 ms sensor reads.
// Polling would drop counts; the interrupt never blinks. (Day 8's lesson:
// the counters must be volatile — the ISR writes them behind main's back.)

#include "hw_encoders.h"
#include "tuning.h"
#include "pins.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

static volatile int32_t count_left, count_right;
static uint8_t state_left, state_right;   // last (prev<<2)|curr, per wheel

// index = (prev_AB << 2) | curr_AB  →  -1, 0, +1
static const int8_t QDEC[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0,
};

static void encoder_irq(uint gpio, uint32_t events)
{
    (void)gpio; (void)events;
    // One snapshot of all GPIOs — cheaper than four gpio_get() calls,
    // and both pins of a channel are sampled at the same instant.
    uint32_t all = sio_hw->gpio_in;

    uint8_t r_ab = (uint8_t)(((all >> PIN_ENC_RIGHT_A) & 1) << 1 |
                             ((all >> PIN_ENC_RIGHT_B) & 1));
    uint8_t l_ab = (uint8_t)(((all >> PIN_ENC_LEFT_A) & 1) << 1 |
                             ((all >> PIN_ENC_LEFT_B) & 1));

    state_right = (uint8_t)(((state_right << 2) | r_ab) & 0xF);
    state_left  = (uint8_t)(((state_left  << 2) | l_ab) & 0xF);

    count_right += QDEC[state_right];
    count_left  += QDEC[state_left];
}

void hw_encoders_init(void)
{
    const uint pins[4] = { PIN_ENC_RIGHT_A, PIN_ENC_RIGHT_B,
                           PIN_ENC_LEFT_A,  PIN_ENC_LEFT_B };
    for (int i = 0; i < 4; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
    }

    // Seed the state from the pins' current level so the first real edge
    // doesn't decode against garbage.
    uint32_t all = sio_hw->gpio_in;
    state_right = (uint8_t)(((all >> PIN_ENC_RIGHT_A) & 1) << 1 |
                            ((all >> PIN_ENC_RIGHT_B) & 1));
    state_left  = (uint8_t)(((all >> PIN_ENC_LEFT_A) & 1) << 1 |
                            ((all >> PIN_ENC_LEFT_B) & 1));

    // First call installs the one-and-only GPIO callback; the rest just
    // enable more pins onto it.
    gpio_set_irq_enabled_with_callback(PIN_ENC_RIGHT_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq);
    gpio_set_irq_enabled(PIN_ENC_RIGHT_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_LEFT_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_LEFT_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// 32-bit reads are a single instruction on Cortex-M0+ — atomic vs the ISR.
int32_t hw_encoder_left(void)  { return ENC_SIGN_LEFT  * count_left; }
int32_t hw_encoder_right(void) { return ENC_SIGN_RIGHT * count_right; }

void hw_encoders_reset(void)
{
    count_left = 0;
    count_right = 0;
}
