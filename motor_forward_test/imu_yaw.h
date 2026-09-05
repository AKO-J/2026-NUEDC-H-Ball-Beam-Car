#ifndef IMU_YAW_H_
#define IMU_YAW_H_

#include <stdbool.h>
#include <stdint.h>

/* ICM-42688-P is explicitly configured for +/-500 dps at 200 Hz. */
#define IMU_YAW_GYRO_LSB_PER_DPS_X10       655
#define IMU_YAW_SCALE_NUM                 1000
#define IMU_YAW_SCALE_DEN                 1000
#define IMU_YAW_WARMUP_MS                 1000U
#define IMU_YAW_BIAS_SAMPLES               256U
#define IMU_YAW_BIAS_SAMPLE_PERIOD_MS        5U
/* 98 / 65.5 = about 1.5 degree/s. */
#define IMU_YAW_STATIONARY_RAW_THRESHOLD     98

typedef struct {
    int32_t headingMdeg;
    int32_t biasRaw;
    int32_t correctedGyroRaw;
    int16_t rawGyroZ;
    int32_t biasQ8;
    uint32_t lastSystickValue;
    uint32_t sampleCycles;
    uint32_t failures;
    uint8_t healthy;
    uint8_t stationary;
} ImuYaw;

/* Must be called after the ICM itself has initialised and warmed up. */
bool ImuYaw_calibrateStationary(ImuYaw *yaw);

/* Use after the owner has started/restarted SysTick. Does not alter SysTick. */
void ImuYaw_rebaseClock(ImuYaw *yaw);

/*
 * Read one gyro sample and update signed yaw. Bias may only adapt when the
 * owner proves the motors are released and the encoders are stationary.
 */
bool ImuYaw_update(ImuYaw *yaw, bool allowBiasUpdate);

#endif /* IMU_YAW_H_ */
