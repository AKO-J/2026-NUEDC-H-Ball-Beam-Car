/*
 * Low-speed straight encoder-distance calibration.
 *
 * Place the car at a measured start mark, keep the path clear, then press
 * PB7. Both wheel encoders are zeroed at that moment. Each wheel is released
 * independently when its absolute quadrature count reaches 6000, so the user
 * can measure the resulting physical travel and calculate counts/cm.
 */

#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "encoder.h"
#include "motor_driver.h"
#include "ssd1306_oled.h"

#define CAL_START_BUTTON_PORT               GPIOB
#define CAL_START_BUTTON_PIN                DL_GPIO_PIN_7
#define CAL_START_BUTTON_IOMUX              IOMUX_PINCM24
#define CAL_START_BUTTON_DEBOUNCE_MS        30U
#define CAL_START_BUTTON_POLL_MS            10U

#define CAL_MCLK_CYCLES_PER_MS           32000U
#define CAL_TARGET_COUNTS                 6000U
#define CAL_RUN_TIMEOUT_MS               12000U
#define CAL_FINAL_BRAKE_MS                 100U

#define CAL_BASE_DUTY_PERCENT               30U
#define CAL_APPROACH_DUTY_PERCENT           18U
#define CAL_CATCHUP_DUTY_PERCENT            18U
#define CAL_MIN_DUTY_PERCENT                18U
#define CAL_MAX_DUTY_PERCENT                45U

#define CAL_CONTROL_PERIOD_MS               20U
#define CAL_APPROACH_PERIOD_MS               2U
#define CAL_APPROACH_WINDOW_COUNTS        1200U

#define CAL_SPEED_ERROR_DEADBAND_COUNTS       3
#define CAL_INTEGRAL_LIMIT_COUNTS            400
#define CAL_PROPORTIONAL_DIVISOR_COUNTS       10
#define CAL_INTEGRAL_DIVISOR_COUNTS           40
#define CAL_MAX_DUTY_CORRECTION               12

/* CCS Expressions / Watch results. */
volatile int32_t g_encoder_cal_left_count;
volatile int32_t g_encoder_cal_right_count;
volatile uint32_t g_encoder_cal_left_progress;
volatile uint32_t g_encoder_cal_right_progress;
volatile int32_t g_encoder_cal_left_delta;
volatile int32_t g_encoder_cal_right_delta;
volatile int32_t g_encoder_cal_speed_error;
volatile int32_t g_encoder_cal_integral_error;
volatile int32_t g_encoder_cal_duty_correction;
volatile uint8_t g_encoder_cal_left_duty;
volatile uint8_t g_encoder_cal_right_duty;
volatile uint32_t g_encoder_cal_elapsed_ms;
volatile uint8_t g_encoder_cal_completed;
volatile uint8_t g_encoder_cal_timed_out;
volatile uint8_t g_encoder_cal_aborted;

static void wait_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_cycles(CAL_MCLK_CYCLES_PER_MS);
    }
}

static uint32_t count_magnitude(int32_t count)
{
    return (count < 0) ? (uint32_t) (-count) : (uint32_t) count;
}

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint8_t duty_from_i32(int32_t duty)
{
    return (uint8_t) clamp_i32(
        duty, (int32_t) CAL_MIN_DUTY_PERCENT,
        (int32_t) CAL_MAX_DUTY_PERCENT);
}

static uint32_t remaining_counts(uint32_t progress)
{
    return (progress >= CAL_TARGET_COUNTS) ? 0U :
           CAL_TARGET_COUNTS - progress;
}

static void format_count_line(char *line, const char *label, int32_t value)
{
    char reverseDigits[10];
    uint32_t magnitude;
    uint8_t index = 0U;
    uint8_t count = 0U;

    while ((label[index] != '\0') &&
           (index < OLED_TEXT_LINE_MAX_CHARS)) {
        line[index] = label[index];
        index++;
    }
    if (index < OLED_TEXT_LINE_MAX_CHARS) {
        if (value < 0) {
            line[index++] = '-';
            magnitude = (uint32_t) (-value);
        } else {
            line[index++] = '+';
            magnitude = (uint32_t) value;
        }
    } else {
        magnitude = 0U;
    }

    do {
        reverseDigits[count++] = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude != 0U) && (count < sizeof(reverseDigits)));

    while ((count != 0U) && (index < OLED_TEXT_LINE_MAX_CHARS)) {
        line[index++] = reverseDigits[--count];
    }
    line[index] = '\0';
}

static void start_button_init(void)
{
    DL_GPIO_enablePower(CAL_START_BUTTON_PORT);
    delay_cycles(16U);
    DL_GPIO_initDigitalInputFeatures(
        CAL_START_BUTTON_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static uint8_t start_button_pressed(void)
{
    return (DL_GPIO_readPins(CAL_START_BUTTON_PORT,
                             CAL_START_BUTTON_PIN) == 0U) ? 1U : 0U;
}

static void wait_for_start_button(void)
{
    uint16_t pressedMs = 0U;
    uint16_t releasedMs = 0U;
    static const char readyLines[OLED_TEXT_LINE_COUNT]
                                [OLED_TEXT_LINE_MAX_CHARS + 1U] = {
        "ENC DIST CAL",
        "TARGET 6000 COUNTS",
        "PLACE ON STRAIGHT",
        "MARK START POSITION",
        "PB7 START",
        "AUTO STOP",
        "MEASURE FINAL CM",
        "KEEP PATH CLEAR",
    };

    while (start_button_pressed() != 0U) {
        Oled_updateTextLines(readyLines, CAL_START_BUTTON_POLL_MS);
        wait_ms(CAL_START_BUTTON_POLL_MS);
    }
    while (pressedMs < CAL_START_BUTTON_DEBOUNCE_MS) {
        if (start_button_pressed() != 0U) {
            pressedMs = (uint16_t) (pressedMs +
                                     CAL_START_BUTTON_POLL_MS);
        } else {
            pressedMs = 0U;
        }
        Oled_updateTextLines(readyLines, CAL_START_BUTTON_POLL_MS);
        wait_ms(CAL_START_BUTTON_POLL_MS);
    }
    /* Start on release, then reuse PB7 as an immediate abort input. */
    while (releasedMs < CAL_START_BUTTON_DEBOUNCE_MS) {
        if (start_button_pressed() == 0U) {
            releasedMs = (uint16_t) (releasedMs +
                                      CAL_START_BUTTON_POLL_MS);
        } else {
            releasedMs = 0U;
        }
        Oled_updateTextLines(readyLines, CAL_START_BUTTON_POLL_MS);
        wait_ms(CAL_START_BUTTON_POLL_MS);
    }
}

static void update_oled(OledStatus *status, uint8_t leftDuty,
                        uint8_t rightDuty, uint16_t elapsedMs)
{
    status->leftPwm = leftDuty;
    status->rightPwm = rightDuty;
    status->driveCommand =
        ((leftDuty == 0U) && (rightDuty == 0U)) ? 0U : 1U;
    status->arcPhase = (g_encoder_cal_completed != 0U) ? 4U :
                       (((g_encoder_cal_timed_out != 0U) ||
                         (g_encoder_cal_aborted != 0U)) ? 5U : 1U);
    status->arcNumber = g_encoder_cal_completed;
    status->fixedProfile = g_encoder_cal_timed_out;
    status->leftEncoder = g_motorA_encoder_count;
    status->rightEncoder = g_motorB_encoder_count;
    status->runElapsedMs = g_encoder_cal_elapsed_ms;
    Oled_updateStatus(status, elapsedMs);
}

int main(void)
{
    OledStatus oledStatus = {0};
    char resultLines[OLED_TEXT_LINE_COUNT]
                    [OLED_TEXT_LINE_MAX_CHARS + 1U] = {
        "ENC DIST RESULT",
        "",
        "",
        "DONE0 TIME0 ABORT0",
        "MEASURE SAME POINT",
        "RESET FOR NEXT RUN",
        "",
        "",
    };
    int32_t integralError = 0;
    uint8_t leftDuty = CAL_BASE_DUTY_PERCENT;
    uint8_t rightDuty = CAL_BASE_DUTY_PERCENT;

    Motor_init();
    Encoder_init();
    start_button_init();
    Motor_stopAll();
    (void) Oled_init();

    wait_for_start_button();
    Encoder_resetCounts();

    while (g_encoder_cal_elapsed_ms < CAL_RUN_TIMEOUT_MS) {
        const int32_t leftBefore = g_motorA_encoder_count;
        const int32_t rightBefore = g_motorB_encoder_count;
        const uint32_t leftBeforeProgress = count_magnitude(leftBefore);
        const uint32_t rightBeforeProgress = count_magnitude(rightBefore);
        const uint32_t leftRemaining = remaining_counts(leftBeforeProgress);
        const uint32_t rightRemaining = remaining_counts(rightBeforeProgress);
        uint32_t periodMs = CAL_CONTROL_PERIOD_MS;
        uint8_t commandLeft = 0U;
        uint8_t commandRight = 0U;

        if (start_button_pressed() != 0U) {
            g_encoder_cal_aborted = 1U;
            break;
        }
        if ((leftRemaining == 0U) && (rightRemaining == 0U)) {
            g_encoder_cal_completed = 1U;
            break;
        }

        if (((leftRemaining != 0U) &&
             (leftRemaining <= CAL_APPROACH_WINDOW_COUNTS)) ||
            ((rightRemaining != 0U) &&
             (rightRemaining <= CAL_APPROACH_WINDOW_COUNTS))) {
            periodMs = CAL_APPROACH_PERIOD_MS;
        }

        if (leftRemaining != 0U) {
            commandLeft = (rightRemaining == 0U) ?
                CAL_CATCHUP_DUTY_PERCENT :
                ((leftRemaining <= CAL_APPROACH_WINDOW_COUNTS) ?
                    CAL_APPROACH_DUTY_PERCENT : leftDuty);
        }
        if (rightRemaining != 0U) {
            commandRight = (leftRemaining == 0U) ?
                CAL_CATCHUP_DUTY_PERCENT :
                ((rightRemaining <= CAL_APPROACH_WINDOW_COUNTS) ?
                    CAL_APPROACH_DUTY_PERCENT : rightDuty);
        }

        Motor_runBothWithDutyFor(
            MOTOR_DIRECTION_FORWARD, MOTOR_DIRECTION_FORWARD,
            commandLeft, commandRight, periodMs);
        Encoder_sample();

        {
            const uint32_t leftAfterProgress =
                count_magnitude(g_motorA_encoder_count);
            const uint32_t rightAfterProgress =
                count_magnitude(g_motorB_encoder_count);
            int32_t speedError =
                ((int32_t) leftAfterProgress -
                 (int32_t) leftBeforeProgress) -
                ((int32_t) rightAfterProgress -
                 (int32_t) rightBeforeProgress);

            g_encoder_cal_left_delta =
                (int32_t) leftAfterProgress -
                (int32_t) leftBeforeProgress;
            g_encoder_cal_right_delta =
                (int32_t) rightAfterProgress -
                (int32_t) rightBeforeProgress;

            if ((leftRemaining != 0U) && (rightRemaining != 0U)) {
                int32_t correction;
                const int32_t referenceDuty =
                    ((leftRemaining <= CAL_APPROACH_WINDOW_COUNTS) ||
                     (rightRemaining <= CAL_APPROACH_WINDOW_COUNTS)) ?
                    (int32_t) CAL_APPROACH_DUTY_PERCENT :
                    (int32_t) CAL_BASE_DUTY_PERCENT;

                if ((speedError >= -CAL_SPEED_ERROR_DEADBAND_COUNTS) &&
                    (speedError <= CAL_SPEED_ERROR_DEADBAND_COUNTS)) {
                    speedError = 0;
                }
                integralError = clamp_i32(
                    ((integralError * 7) / 8) + speedError,
                    -CAL_INTEGRAL_LIMIT_COUNTS,
                    CAL_INTEGRAL_LIMIT_COUNTS);
                correction = clamp_i32(
                    (speedError / CAL_PROPORTIONAL_DIVISOR_COUNTS) +
                    (integralError / CAL_INTEGRAL_DIVISOR_COUNTS),
                    -CAL_MAX_DUTY_CORRECTION,
                    CAL_MAX_DUTY_CORRECTION);
                leftDuty = duty_from_i32(referenceDuty - correction);
                rightDuty = duty_from_i32(referenceDuty + correction);

                g_encoder_cal_speed_error = speedError;
                g_encoder_cal_integral_error = integralError;
                g_encoder_cal_duty_correction = correction;
            }

            g_encoder_cal_left_count = g_motorA_encoder_count;
            g_encoder_cal_right_count = g_motorB_encoder_count;
            g_encoder_cal_left_progress = leftAfterProgress;
            g_encoder_cal_right_progress = rightAfterProgress;
        }

        g_encoder_cal_left_duty = commandLeft;
        g_encoder_cal_right_duty = commandRight;
        g_encoder_cal_elapsed_ms += periodMs;
        update_oled(&oledStatus, commandLeft, commandRight,
                    (uint16_t) periodMs);
    }

    if ((g_encoder_cal_completed == 0U) &&
        (g_encoder_cal_aborted == 0U)) {
        g_encoder_cal_timed_out = 1U;
    }
    /* Capture wheel motion during the brief short-brake, not only the counts
     * observed just before motor power was removed. */
    Motor_brakeAllFor(CAL_FINAL_BRAKE_MS);
    Encoder_sample();

    for (;;) {
        Encoder_sample();
        g_encoder_cal_left_count = g_motorA_encoder_count;
        g_encoder_cal_right_count = g_motorB_encoder_count;
        g_encoder_cal_left_progress =
            count_magnitude(g_motorA_encoder_count);
        g_encoder_cal_right_progress =
            count_magnitude(g_motorB_encoder_count);
        format_count_line(
            resultLines[1], "LEFT ", g_encoder_cal_left_count);
        format_count_line(
            resultLines[2], "RIGHT ", g_encoder_cal_right_count);
        resultLines[3][4] =
            (g_encoder_cal_completed != 0U) ? '1' : '0';
        resultLines[3][10] =
            (g_encoder_cal_timed_out != 0U) ? '1' : '0';
        resultLines[3][17] =
            (g_encoder_cal_aborted != 0U) ? '1' : '0';
        Oled_updateTextLines(resultLines, CAL_START_BUTTON_POLL_MS);
        wait_ms(CAL_START_BUTTON_POLL_MS);
    }
}
