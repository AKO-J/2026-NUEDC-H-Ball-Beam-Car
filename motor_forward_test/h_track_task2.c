#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "encoder.h"
#include "h_track_controller.h"
#include "h_track_finish.h"
#include "h_track_pwm_config.h"
#include "icm42688.h"
#include "imu_yaw.h"
#include "ir_line_sensor.h"
#include "motor_driver.h"
#include "speed_matcher.h"
#include "ssd1306_oled.h"

/*
 * LF04 native four-channel line following with encoder-distance parking.
 *
 * LF04 is a 940-nm comparator board.  With the documented 3.3-V supply it
 * reports white=1 and black=0.  Its sensor-side order is O4,O3,O2,O1, so O1
 * is physically right with the documented downward-facing installation.
 * LF04 only follows the route. The A cross-line is not detected reliably, so
 * separately calibrated left/right encoder distances are averaged and the
 * car stops after one 6.1416-m centre-line lap.
 */
#define H_TRACK_IR_BLACK_LEVEL_HIGH             0U
#define H_TRACK_IR_O1_IS_LEFT                   0U

#define H_TRACK_ENABLE_OLED                     1U
#define H_TRACK_START_BUTTON_PORT               GPIOB
#define H_TRACK_START_BUTTON_PIN                DL_GPIO_PIN_7
#define H_TRACK_START_BUTTON_IOMUX              IOMUX_PINCM24
#define H_TRACK_START_BUTTON_DEBOUNCE_MS        30U
#define H_TRACK_START_BUTTON_POLL_MS            10U

#define H_TRACK_CONTROL_SLICE_MS                 4U
#define H_TRACK_RUN_TIMEOUT_MS               20000U
#define H_TRACK_MCLK_CYCLES_PER_MS          32000U
#define H_TRACK_TIME_RELOAD_CYCLES       0x00FFFFFFU
#define H_TRACK_STATE_ELAPSED_MAX_MS        100U
#define H_TRACK_SPEED_WINDOW_MS              20U
/* ICM output and host yaw integration both target 200 Hz. */
#define H_TRACK_IMU_SAMPLE_PERIOD_MS          5U
#define H_TRACK_IMU_START_RETRIES             3U
#define H_TRACK_IMU_RETRY_GAP_MS            100U
#define H_TRACK_IMU_ENCODER_STILL_COUNTS       2U
#ifndef H_TRACK_TEST_TIMED_LAP
#define H_TRACK_TEST_TIMED_LAP                0U
#endif
#ifndef H_TRACK_TEST_STAGED_YAW_STOP
#define H_TRACK_TEST_STAGED_YAW_STOP           0U
#endif
#define H_TRACK_TIMED_LAP_STOP_MS         12000U
#define H_TRACK_STAGED_SLOW1_MDEG        145000L
#define H_TRACK_STAGED_SLOW2_MDEG        160000L
#define H_TRACK_STAGED_STOP_MDEG         190000L
#define H_TRACK_STAGED_SLOW1_DUTY_PERCENT     75U
#define H_TRACK_STAGED_SLOW2_DUTY_PERCENT     50U

#if H_TRACK_TEST_STAGED_YAW_STOP != 0U
#define H_TRACK_ACTIVE_YAW_STOP_TARGET_MDEG H_TRACK_STAGED_STOP_MDEG
#else
#define H_TRACK_ACTIVE_YAW_STOP_TARGET_MDEG      0L
#endif

/*
 * With the corrected physical forward polarity, nominal differential-drive
 * ordering is correct: to turn toward a line on the left, slow the physical
 * left wheel and speed up the physical right wheel (and vice versa).
 */
#define H_TRACK_TURN_OUTPUT_INVERTED            0U

/* ---- CCS Watch / OLED diagnostics ---- */
volatile uint8_t g_line_raw_mask;
volatile uint8_t g_line_ir_black_mask;
volatile uint8_t g_line_black_mask;
volatile uint8_t g_line_left_to_right_mask;
volatile uint8_t g_line_filtered_mask;
volatile uint8_t g_line_candidate_mask;
volatile uint16_t g_line_confirm_ms;
volatile int16_t g_line_error;
volatile uint8_t g_line_requested_left_duty;
volatile uint8_t g_line_requested_right_duty;
volatile uint8_t g_line_applied_left_duty;
volatile uint8_t g_line_applied_right_duty;
volatile uint8_t g_line_inner_target_duty;
volatile uint8_t g_line_outer_target_duty;
volatile uint8_t g_line_mode;
volatile uint32_t g_line_control_steps;

volatile uint32_t g_h_track_run_elapsed_ms;
volatile uint8_t g_h_track_run_timeout_reached;
volatile uint8_t g_h_track_stage;
volatile uint8_t g_h_track_dh_white_state;
volatile uint32_t g_h_track_distance_counts;
volatile uint32_t g_h_track_distance_cm_x100;
volatile uint32_t g_h_track_finish_remaining_cm_x100;
volatile uint8_t g_h_track_finish_slow_armed;
volatile uint8_t g_h_track_brake_applied;
volatile uint32_t g_h_track_marker_detected_counts;
volatile uint32_t g_h_track_stop_target_counts;
volatile uint16_t g_h_track_marker_confirm_ms;
volatile int8_t g_h_track_last_turn_direction;
volatile int32_t g_h_track_speed_error;
volatile int8_t g_h_track_speed_trim;
volatile uint8_t g_h_track_imu_healthy;
volatile uint8_t g_h_track_imu_who_am_i;
volatile uint8_t g_h_track_imu_last_error;
volatile int16_t g_h_track_gyro_z_raw;
volatile int32_t g_h_track_heading_mdeg;
volatile int32_t g_h_track_gyro_z_bias;
volatile int32_t g_h_track_gyro_z_corrected;
volatile uint32_t g_h_track_gyro_sample_cycles;
volatile uint32_t g_h_track_imu_failures;
volatile uint8_t g_h_track_gyro_stationary;
volatile uint8_t g_h_track_gyro_bias_update_allowed;
volatile uint8_t g_h_track_imu_calibrated;
volatile uint8_t g_h_track_yaw_a_marker_armed;
volatile uint8_t g_h_track_yaw_target_reached;
volatile uint16_t g_h_track_yaw_confirm_ms;
volatile uint8_t g_h_track_imu_fallback_active;
volatile uint8_t g_h_track_stop_latched;
volatile uint8_t g_h_track_timed_stop_reached;
volatile int32_t g_h_track_heading_at_stop_mdeg;
volatile uint8_t g_h_track_slowdown_active;
volatile uint8_t g_h_track_emergency_stop;

static uint32_t s_time_last_tick;
static uint8_t s_imu_started;
static ImuYaw s_imu_yaw;
static int32_t s_imu_last_left_encoder;
static int32_t s_imu_last_right_encoder;

static const HTrackConfig k_track_config = {
    .centerLeftDuty = H_TRACK_LF04_CENTER_LEFT_DUTY,
    .centerRightDuty = H_TRACK_LF04_CENTER_RIGHT_DUTY,
    .smallInnerDuty = H_TRACK_LF04_SMALL_INNER_DUTY,
    .smallOuterDuty = H_TRACK_LF04_SMALL_OUTER_DUTY,
    .bigInnerDuty = H_TRACK_LF04_BIG_INNER_DUTY,
    .bigOuterDuty = H_TRACK_LF04_BIG_OUTER_DUTY,
    .sharpInnerDuty = H_TRACK_LF04_SHARP_INNER_DUTY,
    .sharpOuterDuty = H_TRACK_LF04_SHARP_OUTER_DUTY,
    .approachDuty = H_TRACK_LF04_APPROACH_DUTY,
    .finishDuty = H_TRACK_LF04_FINISH_DUTY,
    .markerConfirmMs = H_TRACK_LF04_MARKER_CONFIRM_MS,
    /* Physical A-marker parking is disabled in the application below. */
    .lapArmCounts = 0U,
    .stopOffsetCounts = H_TRACK_LF04_ENCODER_COUNTS_PER_CM *
                        H_TRACK_LF04_STOP_OFFSET_CM,
    .stopSlowWindowCounts = H_TRACK_LF04_ENCODER_COUNTS_PER_CM *
                            H_TRACK_LF04_STOP_SLOW_WINDOW_CM,
};

static const HTrackFinishConfig k_finish_config = {
    .slowStartDistance = H_TRACK_ENCODER_LAP_STOP_CM_X100 -
                         H_TRACK_ENCODER_SLOW_WINDOW_CM_X100,
    .stopDistance = H_TRACK_ENCODER_LAP_STOP_CM_X100,
};

static const SpeedMatcherConfig k_speed_matcher_config = {
    .deadbandCounts = 2U,
    .proportionalDivisorCounts = 8U,
    .integralDivisorCounts = 32U,
    .maxDutyTrim = 8U,
    .integralLimitCounts = 256,
};

static void wait_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_cycles(H_TRACK_MCLK_CYCLES_PER_MS);
    }
}

static void start_button_init(void)
{
    DL_GPIO_enablePower(H_TRACK_START_BUTTON_PORT);
    delay_cycles(16U);
    DL_GPIO_initDigitalInputFeatures(H_TRACK_START_BUTTON_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

static uint8_t start_button_pressed(void)
{
    return (DL_GPIO_readPins(H_TRACK_START_BUTTON_PORT,
                             H_TRACK_START_BUTTON_PIN) == 0U) ? 1U : 0U;
}

static void wait_for_start_button(void)
{
    uint16_t pressedMs = 0U;
#if H_TRACK_ENABLE_OLED != 0U
#if H_TRACK_TEST_TIMED_LAP != 0U
    static const char readyLines[OLED_TEXT_LINE_COUNT]
                                [OLED_TEXT_LINE_MAX_CHARS + 1U] = {
        "TIME LAP CAPTURE",
        "STOP AT 12.000 S",
        "LF04 3V3 30mm",
        "IMU CAL KEEP STILL",
        "PRESS START",
        "LATCH HDG AT STOP",
        "NO A MARKER STOP",
        "WHEELS MUST BE SAFE",
    };
#elif H_TRACK_TEST_STAGED_YAW_STOP != 0U
    static const char readyLines[OLED_TEXT_LINE_COUNT]
                                [OLED_TEXT_LINE_MAX_CHARS + 1U] = {
        "STAGED YAW STOP",
        "145 DEG: PWM 75%",
        "160 DEG: PWM 50%",
        "190 DEG: STOP",
        "IMU CAL KEEP STILL",
        "PRESS START",
        "LATCH TIME + HDG",
        "WHEELS MUST BE SAFE",
    };
#else
    static const char readyLines[OLED_TEXT_LINE_COUNT]
                                [OLED_TEXT_LINE_MAX_CHARS + 1U] = {
        "ENCODER LAP STOP",
        "TARGET 614.2 CM",
        "CAL L64.45 R62.67",
        "START CAR AT A",
        "LF04 FOLLOWS LINE",
        "SLOW LAST 30 CM",
        "PB7 START",
        "IMU IS DIAGNOSTIC",
    };
#endif
#endif

    while (start_button_pressed() != 0U) {
#if H_TRACK_ENABLE_OLED != 0U
        Oled_updateTextLines(readyLines, H_TRACK_START_BUTTON_POLL_MS);
#endif
        wait_ms(H_TRACK_START_BUTTON_POLL_MS);
    }
    while (pressedMs < H_TRACK_START_BUTTON_DEBOUNCE_MS) {
        if (start_button_pressed() != 0U) {
            pressedMs = (uint16_t) (pressedMs +
                                     H_TRACK_START_BUTTON_POLL_MS);
        } else {
            pressedMs = 0U;
        }
#if H_TRACK_ENABLE_OLED != 0U
        Oled_updateTextLines(readyLines, H_TRACK_START_BUTTON_POLL_MS);
#endif
        wait_ms(H_TRACK_START_BUTTON_POLL_MS);
    }
}

static void run_time_init(void)
{
    SysTick->CTRL = 0U;
    SysTick->LOAD = H_TRACK_TIME_RELOAD_CYCLES;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    s_time_last_tick = SysTick->VAL;
}

static uint16_t elapsed_ms(void)
{
    const uint32_t now = SysTick->VAL & H_TRACK_TIME_RELOAD_CYCLES;
    uint32_t elapsedCycles;
    uint32_t milliseconds;

    if (s_time_last_tick >= now) {
        elapsedCycles = s_time_last_tick - now;
    } else {
        elapsedCycles = s_time_last_tick + H_TRACK_TIME_RELOAD_CYCLES + 1U -
                        now;
    }
    s_time_last_tick = now;
    milliseconds = (elapsedCycles + (H_TRACK_MCLK_CYCLES_PER_MS / 2U)) /
                   H_TRACK_MCLK_CYCLES_PER_MS;
    if (milliseconds == 0U) {
        milliseconds = 1U;
    }
    if (milliseconds > H_TRACK_STATE_ELAPSED_MAX_MS) {
        milliseconds = H_TRACK_STATE_ELAPSED_MAX_MS;
    }
    return (uint16_t) milliseconds;
}

static uint32_t abs_count(int32_t count)
{
    return (count < 0) ? (uint32_t) (-count) : (uint32_t) count;
}

static uint32_t forward_distance_counts(void)
{
    const uint32_t left = abs_count(g_motorA_encoder_count);
    const uint32_t right = abs_count(g_motorB_encoder_count);

    return (left / 2U) + (right / 2U) + ((left & right) & 1U);
}

static uint32_t wheel_distance_cm_x100(uint32_t count,
                                       uint32_t countsPer100Cm)
{
    if (count > (UINT32_MAX / 10000U)) {
        return UINT32_MAX;
    }
    return ((count * 10000U) + (countsPer100Cm / 2U)) /
           countsPer100Cm;
}

static uint32_t forward_distance_cm_x100(void)
{
    const uint32_t left = wheel_distance_cm_x100(
        abs_count(g_motorA_encoder_count),
        H_TRACK_LEFT_ENCODER_COUNTS_PER_100_CM);
    const uint32_t right = wheel_distance_cm_x100(
        abs_count(g_motorB_encoder_count),
        H_TRACK_RIGHT_ENCODER_COUNTS_PER_100_CM);

    return (left / 2U) + (right / 2U) + ((left & right) & 1U);
}

static int16_t dh_state_to_error(uint8_t state)
{
    switch (state & 0x0FU) {
    case 0x1U:
    case 0x3U:
        return -100;
    case 0x7U:
        return -60;
    case 0xBU:
        return -25;
    case 0x9U:
        return 0;
    case 0xDU:
        return 25;
    case 0xEU:
        return 60;
    case 0x8U:
    case 0xCU:
        return 100;
    default:
        return 0;
    }
}

static void command_turn_toward_line(int8_t direction,
                                     uint8_t innerDuty, uint8_t outerDuty,
                                     uint8_t *leftDuty, uint8_t *rightDuty)
{
    const uint8_t lineIsLeft = (direction < 0) ? 1U : 0U;
    const uint8_t useNominalOrdering =
        (H_TRACK_TURN_OUTPUT_INVERTED == 0U) ? 1U : 0U;

    if (lineIsLeft == useNominalOrdering) {
        *leftDuty = innerDuty;
        *rightDuty = outerDuty;
    } else {
        *leftDuty = outerDuty;
        *rightDuty = innerDuty;
    }
}

static void command_to_duty(const HTrackCommand *command,
                            uint8_t *leftDuty, uint8_t *rightDuty)
{
    switch (command->drive) {
    case H_TRACK_DRIVE_CENTER:
        *leftDuty = command->innerDuty;
        *rightDuty = command->outerDuty;
        break;
    case H_TRACK_DRIVE_LINE_TURN:
    case H_TRACK_DRIVE_HOLD_LAST:
        command_turn_toward_line(command->direction, command->innerDuty,
                                 command->outerDuty, leftDuty, rightDuty);
        break;
    case H_TRACK_DRIVE_STOP_APPROACH:
        *leftDuty = command->innerDuty;
        *rightDuty = command->outerDuty;
        break;
    case H_TRACK_DRIVE_STOP:
    default:
        *leftDuty = 0U;
        *rightDuty = 0U;
        break;
    }
}

static void update_imu_diagnostic(void)
{
    const int32_t leftEncoder = g_motorA_encoder_count;
    const int32_t rightEncoder = g_motorB_encoder_count;
    const uint8_t pwmReleased =
        ((g_motor_pwm_a_duty == 0U) && (g_motor_pwm_b_duty == 0U)) ? 1U : 0U;
    const uint8_t encodersStill =
        ((abs_count(leftEncoder - s_imu_last_left_encoder) <=
          H_TRACK_IMU_ENCODER_STILL_COUNTS) &&
         (abs_count(rightEncoder - s_imu_last_right_encoder) <=
          H_TRACK_IMU_ENCODER_STILL_COUNTS)) ? 1U : 0U;

    g_h_track_gyro_bias_update_allowed =
        ((pwmReleased != 0U) && (encodersStill != 0U)) ? 1U : 0U;
    s_imu_last_left_encoder = leftEncoder;
    s_imu_last_right_encoder = rightEncoder;

    if (s_imu_started != 0U) {
        (void) ImuYaw_update(
            &s_imu_yaw, g_h_track_gyro_bias_update_allowed != 0U);
    }

    g_h_track_imu_healthy = s_imu_yaw.healthy;
    g_h_track_gyro_z_raw = s_imu_yaw.rawGyroZ;
    g_h_track_heading_mdeg = s_imu_yaw.headingMdeg;
    g_h_track_gyro_z_bias = s_imu_yaw.biasRaw;
    g_h_track_gyro_z_corrected = s_imu_yaw.correctedGyroRaw;
    g_h_track_gyro_sample_cycles = s_imu_yaw.sampleCycles;
    g_h_track_imu_failures = s_imu_yaw.failures;
    g_h_track_gyro_stationary = s_imu_yaw.stationary;
    g_h_track_imu_who_am_i = g_icm42688_who_am_i;
    g_h_track_imu_last_error = g_icm42688_last_error;
}

/*
 * A single NACK during the 256-sample stationary bias collection used to
 * disable yaw for the entire run.  The ICM driver already rebuilds I2C0
 * after a failed transfer, so retry the complete still-car calibration before
 * declaring that the IMU is unavailable for this lap.
 */
static uint8_t start_imu_heading(void)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < H_TRACK_IMU_START_RETRIES; attempt++) {
        if (Icm42688_init()) {
            /* Keep the car still during the sensor warmup and bias average. */
            wait_ms(IMU_YAW_WARMUP_MS);
            if (ImuYaw_calibrateStationary(&s_imu_yaw)) {
                return 1U;
            }
        }

        if ((attempt + 1U) < H_TRACK_IMU_START_RETRIES) {
            wait_ms(H_TRACK_IMU_RETRY_GAP_MS);
        }
    }

    return 0U;
}

#if defined(COMPETITION_UNIFIED_ENTRY)
void Task2_run(void)
#else
int main(void)
#endif
{
    HTrackController controller;
    HTrackFinishDetector finishDetector;
    SpeedMatcher speedMatcher;
    HTrackCommand command;
#if H_TRACK_ENABLE_OLED != 0U
    OledStatus oledStatus = {0};
    oledStatus.functionId = 2U;
#endif
    uint32_t lastSpeedLeft;
    uint32_t lastSpeedRight;
    uint16_t speedWindowMs = 0U;
    uint16_t imuSampleWindowMs = H_TRACK_IMU_SAMPLE_PERIOD_MS;
    uint16_t loopElapsedMs;
    uint16_t runtimeStartReleaseMs = 0U;
    uint8_t runtimeStartLatched = 1U;
    uint8_t leftDuty = 0U;
    uint8_t rightDuty = 0U;

    Motor_init();
    Encoder_init();
    IrLineSensor_init();
    start_button_init();
    Motor_stopAll();
#if H_TRACK_ENABLE_OLED != 0U
    (void) Oled_init();
#endif

    s_imu_started = start_imu_heading();
    g_h_track_imu_calibrated = s_imu_started;
    g_h_track_imu_who_am_i = g_icm42688_who_am_i;
    g_h_track_imu_last_error = g_icm42688_last_error;

    wait_for_start_button();
    Encoder_resetCounts();
    s_imu_last_left_encoder = 0;
    s_imu_last_right_encoder = 0;
    HTrackController_init(&controller, &k_track_config);
    HTrackFinishDetector_init(&finishDetector, &k_finish_config);
    SpeedMatcher_init(&speedMatcher, &k_speed_matcher_config);
    lastSpeedLeft = 0U;
    lastSpeedRight = 0U;
    run_time_init();
    if (s_imu_started != 0U) {
        ImuYaw_rebaseClock(&s_imu_yaw);
    }
    /* The physical A-marker remains disabled. Formal parking is now bounded
     * by encoder distance, so an IMU failure is diagnostic rather than a
     * reason to turn the lap into an open-ended run. */
    g_h_track_imu_fallback_active = (s_imu_started == 0U) ? 1U : 0U;
    g_h_track_yaw_a_marker_armed = 0U;
    g_h_track_stop_target_counts = H_TRACK_ENCODER_APPROX_LAP_COUNTS;
    g_h_track_run_elapsed_ms = 0U;
    g_h_track_emergency_stop = 0U;
    HTrackController_setAMarkerArmed(&controller, 0U);

    for (;;) {
        const uint8_t rawMask = IrLineSensor_readStableRawMask();
        const uint8_t blackConnectorMask =
            IrLineSensor_rawToBlackMask(rawMask, H_TRACK_IR_BLACK_LEVEL_HIGH);
        const uint8_t blackLeftToRightMask =
            IrLineSensor_toLeftToRightMask(blackConnectorMask,
                                           H_TRACK_IR_O1_IS_LEFT);
        const uint8_t dhWhiteState =
            IrLineSensor_rawToDhWhiteState(rawMask, H_TRACK_IR_O1_IS_LEFT);
        const uint32_t distanceCounts = forward_distance_counts();
        const uint32_t distanceCmX100 = forward_distance_cm_x100();

        loopElapsedMs = elapsed_ms();

        /* The start press that launched the lap is already latched. Re-arm
         * only after a stable release; the next S1 edge is an emergency stop.
         * Latching it before the timer update freezes TIME in this slice. */
        if (start_button_pressed() != 0U) {
            runtimeStartReleaseMs = 0U;
            if (runtimeStartLatched == 0U) {
                runtimeStartLatched = 1U;
                g_h_track_emergency_stop = 1U;
                g_h_track_stop_latched = 1U;
            }
        } else if (runtimeStartLatched != 0U) {
            if (runtimeStartReleaseMs < H_TRACK_START_BUTTON_DEBOUNCE_MS) {
                runtimeStartReleaseMs = (uint16_t) (
                    runtimeStartReleaseMs + loopElapsedMs);
            }
            if (runtimeStartReleaseMs >= H_TRACK_START_BUTTON_DEBOUNCE_MS) {
                runtimeStartLatched = 0U;
                runtimeStartReleaseMs = 0U;
            }
        }
        g_line_raw_mask = rawMask;
        g_line_ir_black_mask = blackLeftToRightMask;
        g_line_black_mask = dhWhiteState;
        g_line_left_to_right_mask = blackLeftToRightMask;
        g_line_filtered_mask = dhWhiteState;
        g_line_error = dh_state_to_error(dhWhiteState);
        g_h_track_dh_white_state = dhWhiteState;
        g_h_track_distance_counts = distanceCounts;
        g_h_track_distance_cm_x100 = distanceCmX100;
        g_h_track_finish_remaining_cm_x100 =
            (distanceCmX100 >= H_TRACK_ENCODER_LAP_STOP_CM_X100) ? 0U :
            H_TRACK_ENCODER_LAP_STOP_CM_X100 - distanceCmX100;

        /* IMU heading remains visible for diagnosis but is not the formal
         * endpoint reference. */
        if (imuSampleWindowMs <=
            (uint16_t) (65535U - loopElapsedMs)) {
            imuSampleWindowMs = (uint16_t) (imuSampleWindowMs + loopElapsedMs);
        }
        if (imuSampleWindowMs >= H_TRACK_IMU_SAMPLE_PERIOD_MS) {
            update_imu_diagnostic();
            /* ImuYaw measures actual SysTick time, so do not fabricate a
             * catch-up sample after an occasional long I2C transaction. */
            imuSampleWindowMs = 0U;
        }
#if H_TRACK_TEST_TIMED_LAP == 0U
#if H_TRACK_TEST_STAGED_YAW_STOP != 0U
        if ((s_imu_started != 0U) &&
            (g_h_track_heading_mdeg >= H_TRACK_ACTIVE_YAW_STOP_TARGET_MDEG)) {
            g_h_track_yaw_target_reached = 1U;
            g_h_track_stop_latched = 1U;
            g_h_track_heading_at_stop_mdeg = g_h_track_heading_mdeg;
        }
#else
        if ((g_h_track_stop_latched == 0U) &&
            (HTrackFinishDetector_step(&finishDetector,
                                       distanceCmX100) != 0U)) {
            g_h_track_stop_latched = 1U;
            g_h_track_heading_at_stop_mdeg = g_h_track_heading_mdeg;
        }
        g_h_track_finish_slow_armed = finishDetector.armed;
        g_h_track_yaw_a_marker_armed = 0U;
        g_h_track_yaw_target_reached = 0U;
        g_h_track_yaw_confirm_ms = 0U;
#endif
#endif

#if H_TRACK_TEST_STAGED_YAW_STOP != 0U
        if ((s_imu_started != 0U) &&
            (g_h_track_heading_mdeg >= H_TRACK_STAGED_SLOW1_MDEG)) {
            g_h_track_slowdown_active = 1U;
        }
        if ((s_imu_started != 0U) &&
            (g_h_track_heading_mdeg >= H_TRACK_STAGED_SLOW2_MDEG)) {
            g_h_track_slowdown_active = 2U;
        }
#endif

#if (H_TRACK_TEST_TIMED_LAP == 0U) && \
    (H_TRACK_TEST_STAGED_YAW_STOP == 0U)
        /* Keep following the final curve, but scale both requested wheel
         * duties so the 100-ms brake has little residual travel at A. */
        if ((finishDetector.armed != 0U) &&
            (command.drive != H_TRACK_DRIVE_STOP)) {
            g_h_track_slowdown_active = 1U;
            leftDuty = (uint8_t) (((uint32_t) leftDuty *
                                   H_TRACK_ENCODER_SLOW_DUTY_PERCENT +
                                   50U) / 100U);
            rightDuty = (uint8_t) (((uint32_t) rightDuty *
                                    H_TRACK_ENCODER_SLOW_DUTY_PERCENT +
                                    50U) / 100U);
        }
#endif

        /* Freeze TIME in the same control slice that latches the endpoint. */
        if ((g_h_track_stop_latched == 0U) &&
            (controller.state != H_TRACK_STATE_FINISHED) &&
            (controller.state != H_TRACK_STATE_FAULT)) {
            if (g_h_track_run_elapsed_ms <=
                (uint32_t) (H_TRACK_RUN_TIMEOUT_MS - loopElapsedMs)) {
                g_h_track_run_elapsed_ms += loopElapsedMs;
            } else {
                g_h_track_run_elapsed_ms = H_TRACK_RUN_TIMEOUT_MS;
            }
        }

#if H_TRACK_TEST_TIMED_LAP != 0U
        if ((s_imu_started != 0U) &&
            (g_h_track_stop_latched == 0U) &&
            (g_h_track_run_elapsed_ms >= H_TRACK_TIMED_LAP_STOP_MS)) {
            g_h_track_timed_stop_reached = 1U;
            g_h_track_stop_latched = 1U;
            /* This is the value to photograph/use for the later correction;
             * live HDG continues to update after the wheels are released. */
            g_h_track_heading_at_stop_mdeg = g_h_track_heading_mdeg;
        }
#endif

#if (H_TRACK_TEST_TIMED_LAP != 0U) || \
    (H_TRACK_TEST_STAGED_YAW_STOP != 0U)
        if (s_imu_started == 0U) {
            /* Heading-capture and staged-yaw test images still require IMU. */
            controller.state = H_TRACK_STATE_FAULT;
            command.drive = H_TRACK_DRIVE_STOP;
            command.direction = 0;
            command.innerDuty = 0U;
            command.outerDuty = 0U;
        } else if (g_h_track_stop_latched != 0U) {
#else
        if (g_h_track_stop_latched != 0U) {
#endif
            /* The configured endpoint is latched; the motor block below
             * applies one short brake and then keeps both wheels released. */
            controller.state = H_TRACK_STATE_FINISHED;
            command.drive = H_TRACK_DRIVE_STOP;
            command.direction = 0;
            command.innerDuty = 0U;
            command.outerDuty = 0U;
        } else {
            command = HTrackController_step(&controller, dhWhiteState,
                                             distanceCounts, loopElapsedMs);
        }
        if (g_h_track_run_elapsed_ms >= H_TRACK_RUN_TIMEOUT_MS) {
            g_h_track_run_timeout_reached = 1U;
            controller.state = H_TRACK_STATE_FAULT;
            command.drive = H_TRACK_DRIVE_STOP;
        }

        command_to_duty(&command, &leftDuty, &rightDuty);

#if H_TRACK_TEST_STAGED_YAW_STOP != 0U
        /* Preserve the line-following differential ratio during slowdown;
         * only reduce its absolute speed to shed momentum before stopping. */
        if ((g_h_track_slowdown_active != 0U) &&
            (command.drive != H_TRACK_DRIVE_STOP)) {
            const uint8_t scalePercent =
                (g_h_track_slowdown_active >= 2U) ?
                H_TRACK_STAGED_SLOW2_DUTY_PERCENT :
                H_TRACK_STAGED_SLOW1_DUTY_PERCENT;

            leftDuty = (uint8_t) (((uint32_t) leftDuty *
                                   scalePercent + 50U) / 100U);
            rightDuty = (uint8_t) (((uint32_t) rightDuty *
                                    scalePercent + 50U) / 100U);
        }
#endif

        /* Encoder speed correction applies to equal-speed centre and final
         * straight parking only.  It is deliberately disabled in turns where
         * unequal wheel speeds are the requested manoeuvre. */
        if ((command.drive == H_TRACK_DRIVE_CENTER) ||
            (command.drive == H_TRACK_DRIVE_STOP_APPROACH)) {
            speedWindowMs = (uint16_t) (speedWindowMs + loopElapsedMs);
            if (speedWindowMs >= H_TRACK_SPEED_WINDOW_MS) {
                (void) SpeedMatcher_update(&speedMatcher,
                    (int32_t) (abs_count(g_motorA_encoder_count) - lastSpeedLeft),
                    (int32_t) (abs_count(g_motorB_encoder_count) - lastSpeedRight));
                lastSpeedLeft = abs_count(g_motorA_encoder_count);
                lastSpeedRight = abs_count(g_motorB_encoder_count);
                speedWindowMs = 0U;
            }
            SpeedMatcher_apply(&speedMatcher, leftDuty, rightDuty, 0U, 100U,
                               &leftDuty, &rightDuty);
        } else {
            speedWindowMs = 0U;
            lastSpeedLeft = abs_count(g_motorA_encoder_count);
            lastSpeedRight = abs_count(g_motorB_encoder_count);
            SpeedMatcher_init(&speedMatcher, &k_speed_matcher_config);
        }

        if (command.drive == H_TRACK_DRIVE_STOP) {
            if ((g_h_track_stop_latched != 0U) &&
                (g_h_track_brake_applied == 0U)) {
                Motor_brakeAllFor(H_TRACK_ENCODER_STOP_BRAKE_MS);
                g_h_track_brake_applied = 1U;
            } else {
                Motor_stopAll();
                wait_ms(H_TRACK_CONTROL_SLICE_MS);
            }
        } else {
            Motor_runBothWithDutyFor(MOTOR_DIRECTION_FORWARD,
                                     MOTOR_DIRECTION_FORWARD,
                                     leftDuty, rightDuty,
                                     H_TRACK_CONTROL_SLICE_MS);
        }
        Encoder_sample();

        g_h_track_stage = (uint8_t) controller.state;
        g_h_track_marker_confirm_ms = controller.markerElapsedMs;
        g_h_track_marker_detected_counts = controller.markerDetectedCounts;
        g_h_track_stop_target_counts = H_TRACK_ENCODER_APPROX_LAP_COUNTS;
        g_h_track_last_turn_direction = controller.lastDirection;
        g_h_track_speed_error = speedMatcher.latestSpeedError;
        g_h_track_speed_trim = speedMatcher.dutyTrim;
        g_line_candidate_mask =
            (controller.state == H_TRACK_STATE_CONFIRM_A_MARK) ? 0x0U : 0xFFU;
        g_line_confirm_ms = controller.markerElapsedMs;
        g_line_mode = (uint8_t) command.drive;
        g_line_requested_left_duty = leftDuty;
        g_line_requested_right_duty = rightDuty;
        g_line_applied_left_duty = leftDuty;
        g_line_applied_right_duty = rightDuty;
        g_line_inner_target_duty = command.innerDuty;
        g_line_outer_target_duty = command.outerDuty;
        g_line_control_steps++;

#if H_TRACK_ENABLE_OLED != 0U
        oledStatus.lineError = g_line_error;
        oledStatus.lineCandidateMask = g_line_candidate_mask;
        oledStatus.lineConfirmMs = controller.markerElapsedMs;
        oledStatus.blackMask = dhWhiteState;
        oledStatus.leftPwm = leftDuty;
        oledStatus.rightPwm = rightDuty;
        oledStatus.arcPhase = (uint8_t) controller.state;
        /* A1 means the final 30-cm slow window is active. F1 reports an IMU
         * diagnostic failure; encoder parking remains available. */
        oledStatus.arcNumber = g_h_track_finish_slow_armed;
        oledStatus.arcDirection = controller.lastDirection;
        oledStatus.fixedProfile = g_h_track_imu_fallback_active;
        oledStatus.fixedInnerDuty = command.innerDuty;
        oledStatus.fixedOuterDuty = command.outerDuty;
        oledStatus.driveCommand = (uint8_t) command.drive;
#if H_TRACK_TEST_TIMED_LAP != 0U
        oledStatus.headingMdeg = (g_h_track_stop_latched != 0U) ?
            g_h_track_heading_at_stop_mdeg : g_h_track_heading_mdeg;
        /* OLED's compact target field is formatted in tenths. In this test,
         * TGT+12.0 means the 12.000-s stop target, not an angle. */
        oledStatus.activeTargetMdeg = (int32_t) H_TRACK_TIMED_LAP_STOP_MS;
#elif H_TRACK_TEST_STAGED_YAW_STOP != 0U
        oledStatus.headingMdeg = (g_h_track_stop_latched != 0U) ?
            g_h_track_heading_at_stop_mdeg : g_h_track_heading_mdeg;
        oledStatus.activeTargetMdeg = H_TRACK_ACTIVE_YAW_STOP_TARGET_MDEG;
#else
        oledStatus.headingMdeg = (g_h_track_stop_latched != 0U) ?
            g_h_track_heading_at_stop_mdeg : g_h_track_heading_mdeg;
        /* The compact OLED formatter prints milli-units with one decimal;
         * 614160 therefore appears as TGT+614.2 for the 614.16-cm lap. */
        oledStatus.activeTargetMdeg =
            (int32_t) (H_TRACK_ENCODER_LAP_STOP_CM_X100 * 10U);
#endif
        oledStatus.gyroBias = g_h_track_gyro_z_bias;
        oledStatus.gyroZCorrected = g_h_track_gyro_z_corrected;
        oledStatus.gyroSampleCycles = g_h_track_gyro_sample_cycles;
        oledStatus.imuHealthy = g_h_track_imu_healthy;
        oledStatus.imuFailures = (g_h_track_imu_failures > 99U) ? 99U :
                                 (uint8_t) g_h_track_imu_failures;
        oledStatus.imuWhoAmI = g_h_track_imu_who_am_i;
        oledStatus.imuLastError = g_h_track_imu_last_error;
        oledStatus.gyroZRaw = g_h_track_gyro_z_raw;
        oledStatus.leftEncoder = g_motorA_encoder_count;
        oledStatus.rightEncoder = g_motorB_encoder_count;
        oledStatus.runElapsedMs = g_h_track_run_elapsed_ms;
        Oled_updateStatus(&oledStatus, loopElapsedMs);
#endif
    }
}
