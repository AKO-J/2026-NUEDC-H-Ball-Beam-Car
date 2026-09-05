/*
 * Inertial yaw calibration: no TB6612 initialisation and no motor command.
 *
 * ICM:  VCC=3V3, GND=GND, SDA=PA28, SCL=PA31, CS=3V3, AD0=GND
 * OLED: VCC=3V3, GND=GND, SCL=PB19, SDA=PA15
 *
 * Keep the car completely still for the first 2.3 s after reset. Then rotate
 * it manually through one known 360-degree turn and read HDG on the OLED.
 */
#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "icm42688.h"
#include "imu_yaw.h"
#include "ssd1306_oled.h"

#define YAW_TEST_TICK_MS                  5U
#define YAW_TEST_RETRY_TICKS            100U
#define YAW_TEST_SYSTICK_RELOAD   0x00FFFFFFU

volatile int32_t g_icm_yaw_heading_mdeg;
volatile int32_t g_icm_yaw_bias_raw;
volatile int32_t g_icm_yaw_corrected_raw;
volatile int16_t g_icm_yaw_raw_z;
volatile uint32_t g_icm_yaw_sample_cycles;
volatile uint32_t g_icm_yaw_failures;
volatile uint32_t g_icm_yaw_run_elapsed_ms;
volatile uint8_t g_icm_yaw_healthy;
volatile uint8_t g_icm_yaw_stationary;

static uint32_t s_test_last_systick;

static void wait_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_cycles(32000U);
    }
}

static void systick_init(void)
{
    SysTick->CTRL = 0U;
    SysTick->LOAD = YAW_TEST_SYSTICK_RELOAD;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    s_test_last_systick = SysTick->VAL & YAW_TEST_SYSTICK_RELOAD;
}

static uint16_t elapsed_ms(void)
{
    const uint32_t now = SysTick->VAL & YAW_TEST_SYSTICK_RELOAD;
    uint32_t cycles;
    uint32_t milliseconds;

    if (s_test_last_systick >= now) {
        cycles = s_test_last_systick - now;
    } else {
        cycles = s_test_last_systick + YAW_TEST_SYSTICK_RELOAD + 1U - now;
    }
    s_test_last_systick = now;
    milliseconds = (cycles + 16000U) / 32000U;
    return (milliseconds > 100U) ? 100U : (uint16_t) milliseconds;
}

static uint8_t start_imu_yaw(ImuYaw *yaw)
{
    if (!Icm42688_init()) {
        return 0U;
    }

    /* Warmup plus 256 x 5-ms stationary bias sampling: keep car still. */
    wait_ms(IMU_YAW_WARMUP_MS);
    if (!ImuYaw_calibrateStationary(yaw)) {
        return 0U;
    }
    ImuYaw_rebaseClock(yaw);
    return 1U;
}

int main(void)
{
    ImuYaw yaw = {0};
    OledStatus status = {0};
    uint16_t retryTicks = 0U;
    uint8_t yawReady;

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    /* The elapsed display is an application heartbeat even if ICM wiring fails. */
    systick_init();
    (void) Oled_init();
    yawReady = start_imu_yaw(&yaw);

    while (1) {
        g_icm_yaw_run_elapsed_ms += elapsed_ms();
        if (yawReady != 0U) {
            /* ICM driver already recovers its I2C bus on an isolated error.
             * Preserve the accumulated angle across a missed sample: losing
             * one transaction must not silently run startup calibration again
             * and make HDG jump back to zero during a 360-degree test. */
            /* No motors are active in this test, so quiet samples may refine
             * stationary bias while deliberate hand rotation still integrates. */
            (void) ImuYaw_update(&yaw, true);
        } else if (++retryTicks >= YAW_TEST_RETRY_TICKS) {
            retryTicks = 0U;
            yawReady = start_imu_yaw(&yaw);
        }

        g_icm_yaw_heading_mdeg = yaw.headingMdeg;
        g_icm_yaw_bias_raw = yaw.biasRaw;
        g_icm_yaw_corrected_raw = yaw.correctedGyroRaw;
        g_icm_yaw_raw_z = yaw.rawGyroZ;
        g_icm_yaw_sample_cycles = yaw.sampleCycles;
        g_icm_yaw_failures = yaw.failures;
        g_icm_yaw_healthy = yaw.healthy;
        g_icm_yaw_stationary = yaw.stationary;

        status.headingMdeg = yaw.headingMdeg;
        status.activeTargetMdeg = 360000;
        status.gyroBias = yaw.biasRaw;
        status.gyroZCorrected = yaw.correctedGyroRaw;
        status.gyroZRaw = yaw.rawGyroZ;
        status.gyroSampleCycles = yaw.sampleCycles;
        status.imuHealthy = yaw.healthy;
        status.imuFailures = (yaw.failures > 99U) ? 99U : (uint8_t) yaw.failures;
        status.imuWhoAmI = g_icm42688_who_am_i;
        status.imuLastError = g_icm42688_last_error;
        status.runElapsedMs = g_icm_yaw_run_elapsed_ms;
        Oled_updateStatus(&status, YAW_TEST_TICK_MS);
        wait_ms(YAW_TEST_TICK_MS);
    }
}
