// hw_imu.c — LSM6DSO gyro Z axis over i2c0 (GP4 SDA / GP5 SCL, addr 0x6B).
//
// Register map, cross-checked against the LSM6DSO datasheet and Pololu's own
// driver for this board:
//   WHO_AM_I  (0x0F) reads 0x6C
//   CTRL2_G   (0x11) ODR_G[7:4]=0b0101 → 208 Hz, FS[3:1]=0b110 → ±2000 dps
//   CTRL3_C   (0x12) BDU=0x40, IF_INC=0x04
//   STATUS    (0x1E) bit 1 GDA = new gyro data available
//   OUTZ_L_G  (0x26) little-endian int16, low byte first
// At ±2000 dps full scale the sensitivity is 70 mdps/LSB.
//
// BDU (block data update) is set because the Z reading is two bytes fetched
// in one burst: without it the chip may refresh the high byte between the
// two, producing a value that never existed. IF_INC is what makes that burst
// walk 0x26 → 0x27 instead of re-reading the same register.
//
// Heading is integrated, not measured: angle += rate × dt. Two consequences
// drive the rest of this file. (1) Any constant offset in "rate" integrates
// into unbounded drift, so hw_imu_calibrate() measures the at-rest bias with
// the robot dead still and subtracts it from every sample. (2) dt has to be
// measured rather than assumed from the nominal 208 Hz, because the caller
// polls on its own schedule and a dropped or late sample would otherwise
// silently mis-scale the angle — every sample is timestamped with the µs
// timer. Calibration cannot remove the bias for good either — it moves with
// temperature — so an integrated heading degrades the longer it runs since
// the last hw_imu_calibrate() / hw_imu_reset_angle().
//
// Sign convention: the gyro's Z axis points up out of the board, so
// counterclockwise (a left turn) is positive, and so is the integrated
// heading.

#include "hw_imu.h"
#include "hw_millis.h"
#include "pins.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define IMU_I2C   i2c0
#define IMU_ADDR  0x6B

#define REG_WHO_AM_I  0x0F
#define REG_CTRL2_G   0x11
#define REG_CTRL3_C   0x12
#define REG_STATUS    0x1E
#define REG_OUTZ_L_G  0x26

#define GYRO_MDPS_PER_LSB 70.0f   // ±2000 dps full scale

static float angle_deg;      // integrated heading, degrees, +CCW
static float rate_dps;       // latest sample, bias removed
static float bias_raw;       // at-rest reading, raw LSB
static uint32_t last_us;     // timestamp of the last consumed sample
static bool present;         // WHO_AM_I answered at init

// Single-register write. The return code is not checked: the only writes are
// the two config registers at init, and a failure there surfaces immediately
// as a gyro that never raises its data-ready flag.
static void reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(IMU_I2C, IMU_ADDR, buf, 2, false);
}

// Burst read of n bytes starting at reg. Returns <0 on a bus error.
static int reg_read(uint8_t reg, uint8_t *dst, size_t n)
{
    // Repeated start: write the register address with no STOP (the `true`
    // argument holds the bus), then turn the bus around and read.
    if (i2c_write_blocking(IMU_I2C, IMU_ADDR, &reg, 1, true) < 0) { return -1; }
    return i2c_read_blocking(IMU_I2C, IMU_ADDR, dst, n, false);
}

bool hw_imu_init(void)
{
    i2c_init(IMU_I2C, 400 * 1000);
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    // The board carries its own I2C pull-ups; enabling the weak internal
    // ones as well only sharpens the rising edges.
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);

    uint8_t who = 0;
    if (reg_read(REG_WHO_AM_I, &who, 1) < 0 || who != 0x6C) {
        present = false;
        return false;
    }

    reg_write(REG_CTRL3_C, 0x44);   // BDU + IF_INC
    reg_write(REG_CTRL2_G, 0x5C);   // 208 Hz, ±2000 dps
    sleep_ms(30);                   // ~6 sample periods of settling

    present = true;
    last_us = hw_micros();
    return true;
}

// True when the gyro has a sample the host has not read yet.
static bool sample_ready(void)
{
    uint8_t status = 0;
    if (reg_read(REG_STATUS, &status, 1) < 0) { return false; }
    return (status & 0x02) != 0;    // GDA
}

// Raw gyro Z in LSB, no bias correction and no scaling.
static int16_t read_raw_z(void)
{
    uint8_t b[2] = { 0, 0 };
    reg_read(REG_OUTZ_L_G, b, 2);
    return (int16_t)(b[0] | (b[1] << 8));
}

void hw_imu_calibrate(void)
{
    if (!present) { return; }
    // 256 samples at 208 Hz ≈ 1.2 s of standing still: long enough to
    // average sample noise down, short enough that the robot can plausibly
    // be held motionless for the whole window.
    int32_t sum = 0;
    for (int i = 0; i < 256; i++) {
        while (!sample_ready()) { tight_loop_contents(); }
        sum += read_raw_z();
    }
    bias_raw = (float)sum / 256.0f;
    angle_deg = 0.0f;
    last_us = hw_micros();
}

bool hw_imu_update(void)
{
    if (!present || !sample_ready()) { return false; }

    int16_t raw = read_raw_z();
    uint32_t now = hw_micros();
    uint32_t dt_us = now - last_us;      // unsigned: correct across the wrap
    last_us = now;

    rate_dps = ((float)raw - bias_raw) * (GYRO_MDPS_PER_LSB / 1000.0f);
    angle_deg += rate_dps * (float)dt_us * 1e-6f;
    return true;
}

float hw_imu_angle_deg(void) { return angle_deg; }
float hw_imu_rate_dps(void)  { return rate_dps; }

void hw_imu_reset_angle(void)
{
    angle_deg = 0.0f;
    last_us = hw_micros();
}
