// hw_encoders.c — 4× quadrature decode in a GPIO IRQ.
//
// Right encoder: GP8 (A) / GP9 (B). Left: GP12 (A) / GP13 (B).
// Every edge on any of the four pins fires one interrupt. The handler reads
// both pins of each channel and indexes a 16-entry transition table with
// (previous AB << 2) | current AB. The four legal gray-code steps yield ±1;
// the "impossible" transitions — a diagonal jump, meaning an edge was missed
// or a contact bounced — yield 0, so noise stops the count from advancing
// instead of corrupting it. Decoding all four edges of each cycle is what
// turns the 12-count magnetic disc on the motor shaft into ~358.3 counts per
// wheel revolution through the ~29.86:1 gearbox.
//
// IRQ rather than polling: at speed-run pace the wheels produce an edge
// roughly every 100 µs, while the main loop is off doing ~1.1 ms line-sensor
// reads. Polling would drop counts wholesale. The counters are volatile
// because the ISR writes them behind the main loop's back.

#include "hw_encoders.h"
#include "tuning.h"
#include "pins.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

static volatile int32_t count_left, count_right;
static uint8_t state_left, state_right;   // last (prev<<2)|curr, per wheel

// index = (prev_AB << 2) | curr_AB → -1, 0, +1. Zeros are both the
// no-change cases (index 0, 5, 10, 15) and the illegal diagonals.
static const int8_t QDEC[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0,
};

// Shared handler for all four encoder pins; which pin fired is irrelevant
// because both channels are re-decoded from a single snapshot every time.
static void encoder_irq(uint gpio, uint32_t events)
{
    (void)gpio; (void)events;
    // One read of the whole GPIO input register — cheaper than four
    // gpio_get() calls, and it samples both pins of a channel at the same
    // instant, which a sequence of separate reads would not.
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

    // Seed the state from the pins' current levels so the first real edge
    // decodes against reality instead of a zeroed history.
    uint32_t all = sio_hw->gpio_in;
    state_right = (uint8_t)(((all >> PIN_ENC_RIGHT_A) & 1) << 1 |
                            ((all >> PIN_ENC_RIGHT_B) & 1));
    state_left  = (uint8_t)(((all >> PIN_ENC_LEFT_A) & 1) << 1 |
                            ((all >> PIN_ENC_LEFT_B) & 1));

    // The RP2040 has one GPIO callback for the whole bank: the first call
    // installs it, the rest only add pins to the same handler.
    gpio_set_irq_enabled_with_callback(PIN_ENC_RIGHT_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq);
    gpio_set_irq_enabled(PIN_ENC_RIGHT_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_LEFT_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_LEFT_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// A 32-bit load is a single instruction on Cortex-M0+, so these cannot tear
// against the ISR and need no critical section.
int32_t hw_encoder_left(void)  { return ENC_SIGN_LEFT  * count_left; }
int32_t hw_encoder_right(void) { return ENC_SIGN_RIGHT * count_right; }

void hw_encoders_reset(void)
{
    count_left = 0;
    count_right = 0;
}
