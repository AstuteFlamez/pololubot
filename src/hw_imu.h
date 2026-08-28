// hw_imu.h — LSM6DSO gyro (Z axis) over I2C: heading by integration.
// You built this on Days 13–14 — this is the same idea, hardened.
#ifndef HW_IMU_H
#define HW_IMU_H

#include <stdint.h>
#include <stdbool.h>

// Init the I2C bus + gyro (208 Hz, ±2000 dps). Returns false if the chip
// doesn't answer its WHO_AM_I — the robot can still walk, but can't turn
// accurately; main shows this on the splash.
bool hw_imu_init(void);

// Measure the gyro's at-rest bias. ROBOT MUST BE PERFECTLY STILL (~1.5 s).
// Without this, "0°/s" reads as a slow constant spin and every integrated
// angle drifts.
void hw_imu_calibrate(void);

// Call as often as possible while you care about the angle. Integrates
// angle += rate × dt whenever the gyro has a fresh sample (208 Hz).
// Returns true if a new sample was consumed.
bool hw_imu_update(void);

float hw_imu_angle_deg(void);   // heading since reset; +CCW (left turn)
float hw_imu_rate_dps(void);    // latest turn rate
void  hw_imu_reset_angle(void);

#endif
