// hw_imu.h — LSM6DSO gyro (Z axis) over I2C: heading by integration.
#ifndef HW_IMU_H
#define HW_IMU_H

#include <stdint.h>
#include <stdbool.h>

// Bring up i2c0 at 400 kHz and configure the gyro for 208 Hz output at
// ±2000 dps full scale. Returns false if the chip does not answer its
// WHO_AM_I, in which case every other call in this header is a no-op: the
// angle stays 0 and hw_imu_update() always returns false. A missing IMU is
// not fatal — the robot still drives, it just cannot turn by angle — so main
// reports the result on the splash screen rather than halting.
bool hw_imu_init(void);

// Measure the gyro's at-rest bias and take it as zero. THE ROBOT MUST BE
// COMPLETELY STILL for the duration: this blocks for 256 samples at 208 Hz,
// about 1.2 s. Also zeroes the heading and restarts the dt clock.
// Without it a stationary gyro reads a small constant rate, which integrates
// into steady heading drift. No-op if the IMU is absent, but note it waits
// on the data-ready flag with no timeout, so a bus that dies mid-calibration
// hangs here.
void hw_imu_calibrate(void);

// Poll the gyro once. If a new sample is waiting it is consumed and folded
// into the heading as angle += rate × dt, where dt is measured from the
// previous consumed sample rather than assumed; returns true. Returns false
// (and does nothing) when no sample is ready or the IMU is absent. Never
// blocks. Call it at least as often as the 208 Hz sample rate — samples are
// not queued, so a slow caller loses angle rather than accumulating it.
bool hw_imu_update(void);

// Heading in degrees accumulated since the last hw_imu_calibrate() or
// hw_imu_reset_angle(). Positive is counterclockwise (a left turn), matching
// the gyro's Z axis pointing up out of the board. Not wrapped: multiple
// turns keep accumulating past ±360.
float hw_imu_angle_deg(void);

// Turn rate in degrees per second from the most recent consumed sample,
// bias removed. Same sign convention as the heading.
float hw_imu_rate_dps(void);

// Set the heading back to 0 and restart the dt clock. Does not re-measure
// the bias.
void hw_imu_reset_angle(void);

#endif
