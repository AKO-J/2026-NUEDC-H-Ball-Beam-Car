/*
 * Safe ICM-42688-P first-test program.
 *
 * It configures only I2C0 and board LEDs. It never initializes or drives
 * TB6612 motor pins, so it is safe with the car powered on the bench.
 *
 * Wiring:
 *   VCC -> 3V3, GND -> GND, SDA/MOSI -> PA28,
 *   SCL/SCLK -> PA31, CS -> 3V3, AD0/MISO -> GND.
 * These two pins do not conflict with the current motor/encoder wiring.
 * The test also probes both legal I2C addresses (0x68 and 0x69).
 */

#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "icm42688.h"

#define STATUS_LED_PORT         GPIOB
#define STATUS_LED_BLUE_PIN     DL_GPIO_PIN_22
#define STATUS_LED_GREEN_PIN    DL_GPIO_PIN_27
#define STATUS_LED_BLUE_IOMUX   IOMUX_PINCM50
#define STATUS_LED_GREEN_IOMUX  IOMUX_PINCM58

#define HEARTBEAT_LED_PORT      GPIOA
#define HEARTBEAT_LED_PIN       DL_GPIO_PIN_0
#define HEARTBEAT_LED_IOMUX     IOMUX_PINCM1

#define SUCCESS_ON_CYCLES       2400000U  /* 75 ms at 32 MHz */
#define SUCCESS_OFF_CYCLES      5600000U  /* 175 ms */
#define ERROR_ON_CYCLES         3200000U  /* 100 ms */
#define ERROR_OFF_CYCLES        3200000U  /* 100 ms */
#define ERROR_GAP_CYCLES       12800000U  /* 400 ms */
#define MOTION_RAW_THRESHOLD    800U

/* CCS-visible variables: their values must change when the module is moved. */
volatile Icm42688Sample g_icm42688_latest_sample;
volatile uint32_t g_icm42688_read_count;
volatile uint32_t g_icm42688_read_failures;
volatile uint8_t g_icm42688_motion_detected;
volatile uint8_t g_icm42688_test_ready;

static uint16_t abs_i16(int16_t value)
{
    if (value < 0) {
        return (uint16_t) (-(int32_t) value);
    }
    return (uint16_t) value;
}

static void status_led_init(void)
{
    DL_GPIO_enablePower(STATUS_LED_PORT);
    delay_cycles(16U);
    DL_GPIO_initDigitalOutput(STATUS_LED_BLUE_IOMUX);
    DL_GPIO_initDigitalOutput(STATUS_LED_GREEN_IOMUX);
    DL_GPIO_clearPins(STATUS_LED_PORT,
                      STATUS_LED_BLUE_PIN | STATUS_LED_GREEN_PIN);
    DL_GPIO_enableOutput(STATUS_LED_PORT,
                         STATUS_LED_BLUE_PIN | STATUS_LED_GREEN_PIN);

    /* Independent LED1, active-low. It confirms that the firmware is alive. */
    DL_GPIO_enablePower(HEARTBEAT_LED_PORT);
    delay_cycles(16U);
    DL_GPIO_initDigitalOutput(HEARTBEAT_LED_IOMUX);
    DL_GPIO_setPins(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
    DL_GPIO_enableOutput(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
}

static void status_led_off(void)
{
    DL_GPIO_clearPins(STATUS_LED_PORT,
                      STATUS_LED_BLUE_PIN | STATUS_LED_GREEN_PIN);
}

static void show_error(void)
{
    uint8_t pulse;
    uint8_t count = g_icm42688_last_error;

    /*
     * Blue pulse count identifies the failure without a debugger:
     * 1 = I2C timeout (usually SDA/SCL/pullup wiring), 2 = bus error/NACK,
     * 3 = target answered but WHO_AM_I was not 0x47.
     */
    if ((count == 0U) || (count > 3U)) {
        count = 3U;
    }
    for (pulse = 0U; pulse < count; pulse++) {
        DL_GPIO_setPins(STATUS_LED_PORT, STATUS_LED_BLUE_PIN);
        delay_cycles(ERROR_ON_CYCLES);
        status_led_off();
        delay_cycles(ERROR_OFF_CYCLES);
    }
    delay_cycles(ERROR_GAP_CYCLES);
}

static void show_success(bool moving)
{
    /* Green pulse means a complete, valid six-axis sample was received. */
    DL_GPIO_setPins(STATUS_LED_PORT, STATUS_LED_GREEN_PIN);
    delay_cycles(SUCCESS_ON_CYCLES);
    status_led_off();

    /* A separate blue pulse appears while any gyro axis is changing quickly. */
    if (moving) {
        DL_GPIO_setPins(STATUS_LED_PORT, STATUS_LED_BLUE_PIN);
        delay_cycles(SUCCESS_ON_CYCLES);
        status_led_off();
    }
    delay_cycles(SUCCESS_OFF_CYCLES);
}

static bool sample_is_moving(const Icm42688Sample *sample)
{
    return (abs_i16(sample->gyroX) > MOTION_RAW_THRESHOLD) ||
           (abs_i16(sample->gyroY) > MOTION_RAW_THRESHOLD) ||
           (abs_i16(sample->gyroZ) > MOTION_RAW_THRESHOLD);
}

int main(void)
{
    Icm42688Sample sample;

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    status_led_init();

    /* LED1 lights briefly at boot, even if I2C wiring is incorrect. */
    DL_GPIO_clearPins(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
    delay_cycles(3200000U);
    DL_GPIO_setPins(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);

    g_icm42688_test_ready = Icm42688_init() ? 1U : 0U;

    while (1) {
        if (g_icm42688_test_ready == 0U) {
            show_error();
            g_icm42688_test_ready = Icm42688_init() ? 1U : 0U;
            continue;
        }

        if (!Icm42688_readSample(&sample)) {
            g_icm42688_read_failures++;
            g_icm42688_test_ready = 0U;
            continue;
        }

        g_icm42688_latest_sample = sample;
        g_icm42688_read_count++;
        g_icm42688_motion_detected = sample_is_moving(&sample) ? 1U : 0U;
        show_success(g_icm42688_motion_detected != 0U);
    }
}
