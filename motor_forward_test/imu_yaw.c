#include "imu_yaw.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "icm42688.h"

#define IMU_YAW_MCLK_CYCLES_PER_MS       32000U
#define IMU_YAW_SYSTICK_RELOAD        0x00FFFFFFU
/* Drop a sample rather than integrating across an abnormally long I2C stall. */
#define IMU_YAW_MAX_SAMPLE_CYCLES      1280000U
#define IMU_YAW_BIAS_Q8_SCALE              256
#define IMU_YAW_BIAS_READ_RETRIES            3U
#define IMU_YAW_RUNTIME_READ_RETRIES          3U

static void wait_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_cycles(IMU_YAW_MCLK_CYCLES_PER_MS);
    }
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint32_t elapsed_cycles(ImuYaw *yaw)
{
    const uint32_t now = SysTick->VAL & IMU_YAW_SYSTICK_RELOAD;
    uint32_t elapsed;

    if (yaw->lastSystickValue >= now) {
        elapsed = yaw->lastSystickValue - now;
    } else {
        elapsed = yaw->lastSystickValue + IMU_YAW_SYSTICK_RELOAD + 1U - now;
    }
    yaw->lastSystickValue = now;
    return elapsed;
}

static int32_t raw_to_mdeg(int32_t raw, uint32_t cycles)
{
    if (cycles > IMU_YAW_MAX_SAMPLE_CYCLES) {
        return 0;
    }

    /* raw / 65.5 dps * measured elapsed seconds, in millidegrees. */
    return (int32_t) ((((int64_t) raw * (int64_t) cycles *
                        IMU_YAW_SCALE_NUM) /
                       ((int64_t) IMU_YAW_GYRO_LSB_PER_DPS_X10 *
                        ((int64_t) IMU_YAW_MCLK_CYCLES_PER_MS / 10) *
                        IMU_YAW_SCALE_DEN)));
}

bool ImuYaw_calibrateStationary(ImuYaw *yaw)
{
    int16_t rawGyroZ;
    int64_t total = 0;
    uint16_t index;

    if (yaw == 0) {
        return false;
    }

    yaw->headingMdeg = 0;
    yaw->biasRaw = 0;
    yaw->correctedGyroRaw = 0;
    yaw->rawGyroZ = 0;
    yaw->biasQ8 = 0;
    yaw->sampleCycles = 0U;
    yaw->failures = 0U;
    yaw->healthy = 0U;
    yaw->stationary = 1U;

    for (index = 0U; index < IMU_YAW_BIAS_SAMPLES; index++) {
        uint8_t attempt;
        uint8_t sampled = 0U;

        /* A failed transfer reconstructs I2C0 in the driver.  Accept the
         * recovered retry rather than throwing away a whole 1.28-s bias
         * collection because of one breadboard-wire glitch. */
        for (attempt = 0U; attempt < IMU_YAW_BIAS_READ_RETRIES; attempt++) {
            if (Icm42688_readGyroZ(&rawGyroZ)) {
                sampled = 1U;
                break;
            }
            yaw->failures++;
            wait_ms(IMU_YAW_BIAS_SAMPLE_PERIOD_MS);
        }
        if (sampled == 0U) {
            return false;
        }
        total += rawGyroZ;
        wait_ms(IMU_YAW_BIAS_SAMPLE_PERIOD_MS);
    }

    yaw->biasRaw = (int32_t) (total / (int64_t) IMU_YAW_BIAS_SAMPLES);
    yaw->biasQ8 = yaw->biasRaw * IMU_YAW_BIAS_Q8_SCALE;
    yaw->healthy = 1U;
    return true;
}

void ImuYaw_rebaseClock(ImuYaw *yaw)
{
    if (yaw != 0) {
        yaw->lastSystickValue = SysTick->VAL & IMU_YAW_SYSTICK_RELOAD;
    }
}

bool ImuYaw_update(ImuYaw *yaw, bool allowBiasUpdate)
{
    int16_t rawGyroZ = 0;
    int32_t corrected;
    uint32_t cycles;
    uint8_t attempt;
    uint8_t sampled = 0U;

    if (yaw == 0) {
        return false;
    }

    for (attempt = 0U; attempt < IMU_YAW_RUNTIME_READ_RETRIES; attempt++) {
        if (Icm42688_readGyroZ(&rawGyroZ)) {
            sampled = 1U;
            break;
        }
        yaw->failures++;
    }
    if (sampled == 0U) {
        yaw->healthy = 0U;
        return false;
    }

    /*
     * Measure time between successful samples. A transient failed transfer
     * therefore no longer drops its entire time interval from yaw integration.
     */
    cycles = elapsed_cycles(yaw);

    corrected = (int32_t) rawGyroZ -
                (yaw->biasQ8 / IMU_YAW_BIAS_Q8_SCALE);
    yaw->rawGyroZ = rawGyroZ;
    yaw->correctedGyroRaw = corrected;
    yaw->sampleCycles = cycles;
    yaw->stationary =
        (abs_i32(corrected) <= IMU_YAW_STATIONARY_RAW_THRESHOLD) ? 1U : 0U;

    if ((yaw->stationary != 0U) && allowBiasUpdate) {
        /* Q8 produces a 1/256 bias adaptation rate without float arithmetic. */
        yaw->biasQ8 += corrected;
        yaw->biasRaw = yaw->biasQ8 / IMU_YAW_BIAS_Q8_SCALE;
    } else {
        /*
         * While the car is moving, integrate even sub-threshold rates. This
         * preserves real slow curvature instead of learning it as gyro bias.
         */
        yaw->headingMdeg += raw_to_mdeg(corrected, cycles);
    }
    yaw->healthy = 1U;
    return true;
}
