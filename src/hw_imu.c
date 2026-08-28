// hw_imu.c — LSM6DSO gyro-Z over i2c0 (GP4 SDA / GP5 SCL, addr 0x6B).
//
// Register facts (verified against the LSM6DSO datasheet and Pololu's own
// driver, reference/.../micropython_demo/pololu_3pi_2040_robot/_lib/lsm6dso.py):
//   WHO_AM_I  (0x0F) reads 0x6C
//   CTRL3_C   (0x12) BDU=0x40 (don't tear 16-bit reads), IF_INC=0x04
//   CTRL2_G   (0x11) ODR_G[7:4]=0b0101 → 208 Hz, FS[3:1]=0b110 → ±2000 dps
//   STATUS    (0x1E) bit1 GDA = new gyro data
//   OUTZ_L_G  (0x26) little-endian int16
// At ±2000 dps full scale, sensitivity = 70 mdps/LSB.
//
// Heading comes from INTEGRATION: angle += rate × dt. Two consequences you
// saw on Day 14: (1) any constant bias integrates into steady drift, hence
// hw_imu_calibrate() with the robot dead still; (2) dt must be measured,
// not assumed — we timestamp every sample with the µs timer.
//
// Sign convention: gyro Z points up, so counterclockwise (a LEFT turn)
// is positive. This matches Pololu's gyro_turn.py.

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

static float angle_deg;      // integrated heading
static float rate_dps;       // latest sample, bias-removed
static float bias_raw;       // at-rest reading, raw LSB
static uint32_t last_us;
static bool present;

static void reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(IMU_I2C, IMU_ADDR, buf, 2, false);
}

static int reg_read(uint8_t reg, uint8_t *dst, size_t n)
{
    // Repeated-start read: write the register address with no STOP, then read.
    if (i2c_write_blocking(IMU_I2C, IMU_ADDR, &reg, 1, true) < 0) { return -1; }
    return i2c_read_blocking(IMU_I2C, IMU_ADDR, dst, n, false);
}

bool hw_imu_init(void)
{
    i2c_init(IMU_I2C, 400 * 1000);
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    // Board has its own pull-ups; the internal ones just firm up the edges.
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);

    uint8_t who = 0;
    if (reg_read(REG_WHO_AM_I, &who, 1) < 0 || who != 0x6C) {
        present = false;
        return false;
    }

    reg_write(REG_CTRL3_C, 0x44);   // BDU + IF_INC
    reg_write(REG_CTRL2_G, 0x5C);   // 208 Hz, ±2000 dps
    sleep_ms(30);                   // let the first samples land

    present = true;
    last_us = hw_micros();
    return true;
}

static bool sample_ready(void)
{
    uint8_t status = 0;
    if (reg_read(REG_STATUS, &status, 1) < 0) { return false; }
    return (status & 0x02) != 0;    // GDA
}

static int16_t read_raw_z(void)
{
    uint8_t b[2] = { 0, 0 };
    reg_read(REG_OUTZ_L_G, b, 2);
    return (int16_t)(b[0] | (b[1] << 8));
}

void hw_imu_calibrate(void)
{
    if (!present) { return; }
    // 256 samples at 208 Hz ≈ 1.25 s of standing still.
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
    uint32_t dt_us = now - last_us;      // wrap-safe unsigned subtraction
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
