/*
 * K230 vision -> X42S white-beam closed-loop bring-up image.
 *
 * On power-up the normal image enters WAIT_LEVEL.  UP/DOWN provide only a
 * low-rate guarded manual level adjustment; automatic ball tracking remains
 * blocked.  After the beam is physically level, START accepts that position
 * as the level reference, installs the measured [-132, +132] travel limit,
 * and then enables tracking.  START while ACTIVE immediately returns to the
 * safe WAIT_LEVEL state.
 *
 * This image deliberately contains no mechanical-limit/home-switch logic.
 */

#include <limits.h>
#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "ball_beam_controller.h"
#include "beam_calibration.h"
#include "beam_level_reference.h"
#include "diagnostic_uart.h"
#include "ssd1306_oled.h"
#include "stepper_beam.h"
#include "vision_uart.h"

#ifndef BALL_BEAM_TASK4_BASELINE
#define BALL_BEAM_TASK4_BASELINE 0
#endif

#if BALL_BEAM_TASK4_BASELINE
#include "encoder.h"
#include "icm42688.h"
#include "motor_driver.h"
#include "task4_baseline_profile.h"
#endif

#define BUTTON_PORT                   GPIOB
#define BUTTON_START_PIN              DL_GPIO_PIN_7
#define BUTTON_DOWN_PIN               DL_GPIO_PIN_8
#define BUTTON_UP_PIN                 DL_GPIO_PIN_15
#define BUTTON_PRESET_LIFT_PIN        DL_GPIO_PIN_0
#define BUTTON_LEVEL_ZERO_PIN          DL_GPIO_PIN_20
#define BUTTON_ALL_PINS               (BUTTON_START_PIN | BUTTON_DOWN_PIN | \
                                       BUTTON_UP_PIN | BUTTON_PRESET_LIFT_PIN | \
                                       BUTTON_LEVEL_ZERO_PIN)
#define BUTTON_PRESET_LIFT_IOMUX      IOMUX_PINCM12
#define BUTTON_LEVEL_ZERO_IOMUX       IOMUX_PINCM48
#define BUTTON_UP_IOMUX               IOMUX_PINCM32
#define BUTTON_DOWN_IOMUX             IOMUX_PINCM25
#define BUTTON_TICK_MS                10U
#define BUTTON_DEBOUNCE_TICKS         3U
#define LEVEL_JOG_STEPS               4
#define LEVEL_HOLD_REPEAT_MS          120U
#define LEVEL_MANUAL_JOG_PPS           80U
/* Raw X42S position calibrated by the user: lower mechanical stop POS 0 to
 * the physical white-beam level at POS -565 (X42S display 63.6 degrees).
 * This is a one-shot pre-calibration lift, not a white-beam angle/table POS. */
#define LOWER_STOP_TO_LEVEL_STEPS    (-565)
#define LOWER_STOP_POS_STEPS          0
#define BOOT_SETTLE_MS                250U
#define VISION_PACKET_TIMEOUT_MS      150U
#define VISION_MIN_CONFIDENCE_MILLI   320U
#define BALL_TARGET_X_PIXEL           160
#define DIAGNOSTIC_LOG_PERIOD_MS      50U

/* The pre-reference adjustment guard uses the same calibrated mechanical
 * span as ACTIVE.  It is deliberately not wider than the existing limits. */
#define WAIT_LEVEL_JOG_MIN_STEPS      BEAM_CAL_MIN_POS
#define WAIT_LEVEL_JOG_MAX_STEPS      BEAM_CAL_MAX_POS

/*
 * Build this source with BALL_BEAM_TASK3_SEQUENCE=1 for competition task 3:
 * ball 0 cm -> +5 cm -> -5 cm.  The normal build remains a centre-hold
 * diagnostic so it can still be used while tuning the vision link.
 */
#ifndef BALL_BEAM_TASK3_SEQUENCE
#define BALL_BEAM_TASK3_SEQUENCE       0
#endif

#if BALL_BEAM_TASK3_SEQUENCE
/* K230 full-frame calibration measured on 2026-08-01:
 * x(-5 cm)=84, x(0 cm)=149, x(+5 cm)=208.  K230 transmits offsets from 149. */
#define TASK3_TARGET_PLUS5_PX          59
#define TASK3_TARGET_MINUS5_PX        (-65)
/* Conservative integer bands: + side 11.8 px/cm, - side 13.0 px/cm. */
#define TASK3_TOLERANCE_PLUS5_PX       11
#define TASK3_TOLERANCE_MINUS5_PX      13
#define TASK3_MAX_BEAM_ANGLE_MDEG     800
#define TASK3_POSITIVE_CRUISE_MDEG    800
#define TASK3_POSITIVE_REVERSE_TRIGGER_PX 12
#define TASK3_POSITIVE_NEAR_MDEG       838 /* calibrated interpolation: POS +70 */
#define TASK3_POSITIVE_BRAKE_ZONE_PX    20
#define TASK3_NEGATIVE_CRUISE_MDEG (-828) /* calibrated interpolation: POS -63 */
#define TASK3_NEGATIVE_NEAR_MDEG    (-404) /* calibrated interpolation: POS -40 */
/* (0 mdeg, POS 0) to (+444 mdeg, POS +44): POS +20 -> about +202 mdeg. */
#define TASK3_NEGATIVE_REVERSE_BRAKE_MDEG 565 /* stopping-distance estimate from 720 mdeg run */
#define TASK3_NEGATIVE_BRAKE_ZONE_PX 40
/* Empirical deceleration when reverse-braking at POS +1.  This is
 * a model parameter to identify from stop tests, not vision-derived accel. */
#define TASK3_REVERSE_BRAKE_DECEL_PXPS2 150
#define TASK3_REVERSE_BRAKE_MARGIN_PX   50
#define TASK3_TIMEOUT_MS             4900U /* remain below the 5-s limit */
#endif

/*
 * With the current right-hand pivot and mirrored K230 image, +1 is the
 * expected sign: ball left of target -> positive beam command. Verify first
 * with the ball removed. If the physical response is reversed, change only
 * this constant to -1; do not swap calibration-table signs.
 */
#define BALL_CONTROL_SIGN             1

#if BALL_BEAM_TASK4_BASELINE
/* BallBeamController_init accepts a full-frame pixel and converts it to the
 * K230 offset domain internally.  Full-frame x=160 therefore means offset 0. */
#define TASK4_IMU_SAMPLE_PERIOD_MS     10U
#define TASK4_IMU_RETRY_PERIOD_MS     500U
#define TASK4_IMU_MISSING_RAW (-2147483647L - 1L)
#define TASK4_START_MAX_ERROR_PX       11
#define TASK4_START_CENTER_DWELL_MS   500U
#endif

typedef struct {
    uint32_t stable;
    uint32_t candidate;
    uint8_t debounceTicks;
} Button;

#if !BALL_BEAM_TASK3_SEQUENCE
typedef enum {
    CLOSED_LOOP_WAIT_LEVEL = 0,
    CLOSED_LOOP_JOG_LEFT,
    CLOSED_LOOP_JOG_RIGHT,
    CLOSED_LOOP_JOG_STOP,
    CLOSED_LOOP_PRESET_LIFT,
    CLOSED_LOOP_RETURN_LEVEL,
    CLOSED_LOOP_PRESET_REFUSED,
    CLOSED_LOOP_ZERO_ACCEPTED,
    CLOSED_LOOP_LEVEL_READY,
    CLOSED_LOOP_ACTIVE,
    CLOSED_LOOP_LOST,
    CLOSED_LOOP_LIMIT_FAULT,
    CLOSED_LOOP_TARGET_REJECTED_UNREFERENCED,
    CLOSED_LOOP_TARGET_REJECTED_LIMIT,
    CLOSED_LOOP_TARGET_REJECTED_BUSY,
    CLOSED_LOOP_TARGET_REJECTED_FAULT,
} ClosedLoopState;
#endif

#if BALL_BEAM_TASK3_SEQUENCE
typedef enum {
    TASK3_LEVEL_CAL = 0,
    TASK3_READY,
    TASK3_TO_PLUS5,
    TASK3_TO_MINUS5,
    TASK3_HOLD_MINUS5,
    TASK3_TIMEOUT,
} Task3Phase;

static uint16_t absolute_error(int16_t value, int16_t target)
{
    const int16_t error = (int16_t) (value - target);
    return (uint16_t) (error < 0 ? -error : error);
}

static void task3_set_target(
    BallBeamController *controller,
    int16_t targetOffsetPx)
{
    /* Controller API uses an image X position; K230 reports an offset. */
    BallBeamController_init(
        controller,
        (int16_t) (BALL_BEAM_CAMERA_CENTER_X_PX + targetOffsetPx));
}

static void task3_change_target(
    BallBeamController *controller,
    int16_t targetOffsetPx)
{
    /* Keep the last velocity sample: it is needed to brake at the reversal. */
    controller->targetXOffsetPx = targetOffsetPx;
}

static bool task3_predicts_minus5_overshoot(const BallBeamCommand *command)
{
    int32_t speed;
    int32_t stoppingDistance;
    int32_t predictedStop;

    if ((command == 0) || (command->velocityPxPerS >= 0)) {
        return false;
    }
    speed = -command->velocityPxPerS;
    /* A bad visual frame must not create an unsafe squared overflow. */
    if (speed > 1000) {
        speed = 1000;
    }
    stoppingDistance = speed * speed /
        (2 * TASK3_REVERSE_BRAKE_DECEL_PXPS2);
    predictedStop = (int32_t) command->predictedXOffsetPx -
                    stoppingDistance;
    return predictedStop <= (TASK3_TARGET_MINUS5_PX +
                             TASK3_REVERSE_BRAKE_MARGIN_PX);
}

static bool task3_limit_command(
    BallBeamCommand *command,
    Task3Phase phase,
    int16_t measuredOffsetPx)
{
    int16_t positiveLimit = TASK3_MAX_BEAM_ANGLE_MDEG;
    int16_t negativeLimit = -TASK3_MAX_BEAM_ANGLE_MDEG;

    if (phase == TASK3_TO_PLUS5) {
        if (measuredOffsetPx < TASK3_POSITIVE_REVERSE_TRIGGER_PX) {
            /* The 2026-08-01 run at +0.70..0.715 degree did not overcome
             * static friction.  Use the already validated +0.80-degree task
             * limit as the launch/cruise command; do not raise that limit. */
            command->requestedAngleMdeg = TASK3_POSITIVE_CRUISE_MDEG;
        } else {
            /* Motor reversal takes roughly 0.5 s in the measured mechanism.
             * Start the existing negative cruise angle at x=+12 px so the
             * ball's inertia carries it into, rather than through, +5 cm. */
            command->requestedAngleMdeg = TASK3_NEGATIVE_CRUISE_MDEG;
        }
    }

    /*
     * Travel toward -5 using the alpha-beta position/velocity prediction.
     * If the predicted stop point under the known POS +1 braking action
     * reaches/passes the target margin, brake before the visual position does.
     */
    if (phase == TASK3_TO_MINUS5) {
        if (task3_predicts_minus5_overshoot(command)) {
            /* Near target, POS -40 gives only a small leftward drive.  POS +1 tilt is
             * required to remove the ball's leftward kinetic energy. */
            command->requestedAngleMdeg = TASK3_NEGATIVE_REVERSE_BRAKE_MDEG;
        } else {
            negativeLimit = (measuredOffsetPx >
                (TASK3_TARGET_MINUS5_PX + TASK3_NEGATIVE_BRAKE_ZONE_PX)) ?
                TASK3_NEGATIVE_CRUISE_MDEG : TASK3_NEGATIVE_NEAR_MDEG;
        }
    } else if (phase == TASK3_HOLD_MINUS5) {
        /* Keep the measured reverse-brake tilt applied until the ball really
         * enters the -5 cm tolerance band.  Returning level after only the
         * trigger frame leaves almost all leftward kinetic energy untouched. */
        command->requestedAngleMdeg = TASK3_NEGATIVE_REVERSE_BRAKE_MDEG;
    }
    if (command->requestedAngleMdeg > positiveLimit) {
        command->requestedAngleMdeg = positiveLimit;
    } else if (command->requestedAngleMdeg < negativeLimit) {
        command->requestedAngleMdeg = negativeLimit;
    }
    return BeamCalibration_angleToPos(
        command->requestedAngleMdeg, &command->calibration);
}
#endif

/* CCS-watchable audit values for every commanded interpolation. */
volatile int16_t g_ball_x_offset_px;
volatile int16_t g_ball_requested_beam_angle_mdeg;
volatile int16_t g_ball_target_pos;
volatile int16_t g_ball_cal_angle_low_mdeg;
volatile int16_t g_ball_cal_angle_high_mdeg;
volatile int16_t g_ball_cal_pos_low;
volatile int16_t g_ball_cal_pos_high;
volatile uint16_t g_ball_confidence_milli;
volatile uint32_t g_ball_last_packet_ms;

#if !BALL_BEAM_TASK3_SEQUENCE
static const char *closed_loop_state_name(ClosedLoopState state)
{
    switch (state) {
    case CLOSED_LOOP_WAIT_LEVEL: return "WAIT_LEVEL";
    case CLOSED_LOOP_JOG_LEFT: return "JOG_LEFT";
    case CLOSED_LOOP_JOG_RIGHT: return "JOG_RIGHT";
    case CLOSED_LOOP_JOG_STOP: return "JOG_STOP";
    case CLOSED_LOOP_PRESET_LIFT: return "PRESET_LIFT";
    case CLOSED_LOOP_RETURN_LEVEL: return "RETURN_LEVEL";
    case CLOSED_LOOP_PRESET_REFUSED: return "PRESET_REFUSED";
    case CLOSED_LOOP_ZERO_ACCEPTED: return "ZERO_ACCEPTED";
    case CLOSED_LOOP_LEVEL_READY: return "LEVEL_READY";
    case CLOSED_LOOP_ACTIVE: return "ACTIVE";
    case CLOSED_LOOP_LOST: return "LOST";
    case CLOSED_LOOP_LIMIT_FAULT: return "LIMIT_FAULT";
    case CLOSED_LOOP_TARGET_REJECTED_UNREFERENCED:
        return "TARGET_REJECTED_UNREFERENCED";
    case CLOSED_LOOP_TARGET_REJECTED_LIMIT:
        return "TARGET_REJECTED_LIMIT";
    case CLOSED_LOOP_TARGET_REJECTED_BUSY:
        return "TARGET_REJECTED_BUSY";
    case CLOSED_LOOP_TARGET_REJECTED_FAULT:
    default:
        return "TARGET_REJECTED_FAULT";
    }
}

static ClosedLoopState target_rejection_state(const StepperBeamStatus *motor,
                                              int32_t targetSteps)
{
    if (motor->fault != STEPPER_BEAM_FAULT_NONE) {
        return CLOSED_LOOP_TARGET_REJECTED_FAULT;
    }
    if ((motor->homeState != STEPPER_BEAM_HOME_READY) ||
        (motor->travelLimitsConfigured == 0U)) {
        return CLOSED_LOOP_TARGET_REJECTED_UNREFERENCED;
    }
    if ((targetSteps < motor->travelMinSteps) ||
        (targetSteps > motor->travelMaxSteps)) {
        return CLOSED_LOOP_TARGET_REJECTED_LIMIT;
    }
    return CLOSED_LOOP_TARGET_REJECTED_BUSY;
}

static void reset_control_history(BallBeamController *controller,
                                  BallBeamCommand *command)
{
    BallBeamController_init(controller, BALL_TARGET_X_PIXEL);
    *command = (BallBeamCommand) {0};
    (void) BeamCalibration_angleToPos(0, &command->calibration);
}

static void stop_for_wait_level(void)
{
    StepperBeam_stop();
    /* StepperBeam_stop() disables pulse output.  Re-arm only so manual jog
     * remains available; no tracking target is issued in WAIT_LEVEL. */
    StepperBeam_setArmed(true);
}
#endif

#if BALL_BEAM_TASK3_SEQUENCE
static const char *diagnostic_state(
    const StepperBeamStatus *motor,
    bool linkFresh,
    bool ballIsValid,
    const BallBeamCommand *command)
{
    if ((!linkFresh) || (!ballIsValid)) {
        return "LOST";
    }
    if ((motor->armed == 0U) || (command->requestedAngleMdeg == 0)) {
        return "HOLD";
    }
    if (((command->velocityPxPerS > 2) &&
         (command->requestedAngleMdeg < 0)) ||
        ((command->velocityPxPerS < -2) &&
         (command->requestedAngleMdeg > 0))) {
        return "BRAKE";
    }
    return "ACCEL";
}
#endif

static void wait_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_cycles(32000U);
    }
}

static void button_init(void)
{
    DL_GPIO_enablePower(BUTTON_PORT);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM24,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE); /* START PB7 */
    DL_GPIO_initDigitalInputFeatures(BUTTON_PRESET_LIFT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE); /* PB0 preset */
    DL_GPIO_initDigitalInputFeatures(BUTTON_LEVEL_ZERO_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE); /* PB20 level zero */
    DL_GPIO_initDigitalInputFeatures(BUTTON_UP_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE); /* UP PB15 */
    DL_GPIO_initDigitalInputFeatures(BUTTON_DOWN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE); /* DOWN PB8 */
}

static uint32_t button_pressed(void)
{
    return (~DL_GPIO_readPins(BUTTON_PORT, BUTTON_ALL_PINS)) &
           BUTTON_ALL_PINS;
}

static uint32_t button_update(Button *button, uint32_t pin,
                              uint32_t pressedPins)
{
    const uint32_t sample = pressedPins & pin;
    uint32_t edge = 0U;

    if (sample != button->candidate) {
        button->candidate = sample;
        button->debounceTicks = 0U;
    } else if (button->debounceTicks < BUTTON_DEBOUNCE_TICKS) {
        ++button->debounceTicks;
        if ((button->debounceTicks == BUTTON_DEBOUNCE_TICKS) &&
            (button->stable != button->candidate)) {
            edge = button->candidate & ~button->stable;
            button->stable = button->candidate;
        }
    }
    return edge;
}

static void set_line(
    char line[OLED_TEXT_LINE_MAX_CHARS + 1U],
    const char *text)
{
    uint8_t i = 0U;

    while ((text[i] != '\0') && (i < OLED_TEXT_LINE_MAX_CHARS)) {
        line[i] = text[i];
        ++i;
    }
    line[i] = '\0';
}

static uint8_t append_text(char *line, uint8_t index, const char *text)
{
    while ((*text != '\0') && (index < OLED_TEXT_LINE_MAX_CHARS)) {
        line[index++] = *text++;
    }
    line[index] = '\0';
    return index;
}

static uint8_t append_signed(char *line, uint8_t index, int32_t value)
{
    char digits[11];
    uint8_t count = 0U;
    uint32_t magnitude;

    if (index >= OLED_TEXT_LINE_MAX_CHARS) {
        return index;
    }
    if (value < 0) {
        line[index++] = '-';
        magnitude = (uint32_t) (-value);
    } else {
        line[index++] = '+';
        magnitude = (uint32_t) value;
    }
    do {
        digits[count++] = (char) ('0' + magnitude % 10U);
        magnitude /= 10U;
    } while ((magnitude != 0U) && (count < sizeof(digits)));
    while ((count != 0U) && (index < OLED_TEXT_LINE_MAX_CHARS)) {
        line[index++] = digits[--count];
    }
    line[index] = '\0';
    return index;
}

static void update_oled(
    const StepperBeamStatus *motor,
    const VisionBallMeasurement *vision,
    const BallBeamCommand *command,
    bool linkFresh,
    int32_t levelReferencePos,
    bool levelSaved,
    const char *message,
    uint32_t nowMs)
{
    char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U] = {{0}};
    uint8_t index;

    set_line(lines[0], levelSaved ? "TASK3 READY/CTRL" : "TASK3 LEVEL CAL");
    set_line(lines[1], levelSaved ? "LEVEL READY/CTRL" : "UP/DN ADJUST");
    set_line(lines[2], linkFresh ? "UART3 RX OK" : "UART3 WAIT/TIMEOUT");

    index = append_text(lines[3], 0U, "XOFF ");
    index = append_signed(lines[3], index, vision->xOffsetPx);
    index = append_text(lines[3], index, " C");
    (void) append_signed(lines[3], index, vision->confidenceMilli);

    index = append_text(lines[4], 0U, levelSaved ? "LEVEL " : "RAW POS ");
    (void) append_signed(lines[4], index,
                          levelSaved ? levelReferencePos : motor->positionSteps);
    index = append_text(lines[5], 0U, "CTRL ");
    (void) append_signed(lines[5], index, command->calibration.targetPos);
    index = append_text(lines[6], 0U, "TARGET ");
    (void) append_signed(lines[6], index, motor->targetSteps);
    set_line(lines[7], message);
    Oled_updateTextLines(lines, (uint16_t) nowMs);
}

static bool save_level_reference(int32_t levelReferencePos)
{
    return StepperBeam_acceptManualReference() &&
           StepperBeam_configureTravelLimits(
               levelReferencePos + BEAM_CAL_MIN_POS,
               levelReferencePos + BEAM_CAL_MAX_POS);
}

static bool set_relative_control_target(
    const BeamCalibrationInterpolation *calibration,
    int32_t levelReferencePos)
{
    int32_t motorTargetPos;

    return BeamLevelReference_makeTarget(levelReferencePos,
                                         calibration->targetPos,
                                         &motorTargetPos) &&
           StepperBeam_setTargetSteps(motorTargetPos);
}

#if defined(COMPETITION_UNIFIED_ENTRY) && BALL_BEAM_TASK3_SEQUENCE
void Task3_run(void)
#else
int main(void)
#endif
{
    Button startButton = {0U, 0U, 0U};
    Button upButton = {0U, 0U, 0U};
    Button downButton = {0U, 0U, 0U};
    Button presetLiftButton = {0U, 0U, 0U};
    Button levelZeroButton = {0U, 0U, 0U};
    BallBeamController controller;
    BallBeamCommand command = {0};
    VisionBallMeasurement vision = {0U, 0U, 0, 0U, 1U};
    StepperBeamStatus motor;
    const char *message = "LEVEL CAL START OK";
    uint32_t nowMs = 0U;
    uint32_t lastRxMs = 0U;
    uint32_t lastDiagnosticLogMs = 0U;
    bool haveReceivedPacket = false;
    bool ballIsValid = false;
#if BALL_BEAM_TASK4_BASELINE
    bool haveValidBall = false;
    uint32_t lastValidBallMs = 0U;
    Task4BaselineProfile task4Profile;
    Task4BaselineCommand task4Drive = {TASK4_BASELINE_DISARMED, 0U, false, false};
    Icm42688Sample imuSample = {0};
    bool imuReady = false;
    bool imuSampleValid = false;
    bool haveValidImu = false;
    uint32_t lastImuSampleMs = 0U;
    uint32_t lastValidImuMs = 0U;
    uint32_t lastImuRetryMs = 0U;
    int32_t lastLeftEncoder = 0;
    int32_t lastRightEncoder = 0;
    int32_t leftSpeedCountsPerS = 0;
    int32_t rightSpeedCountsPerS = 0;
    uint32_t task4CenterStableSinceMs = 0U;
    bool task4CenterStable = false;
#endif
#if BALL_BEAM_TASK3_SEQUENCE
    Task3Phase task3Phase = TASK3_LEVEL_CAL;
    uint32_t task3StartedMs = 0U;
    int32_t levelReferencePos = 0;
    bool levelSaved = false;
    bool lowerStopLiftDone = false;
    uint32_t nextLevelJogMs = 0U;
#else
    int32_t levelReferencePos = 0;
    bool levelSaved = false;
    ClosedLoopState closedLoopState = CLOSED_LOOP_WAIT_LEVEL;
    uint32_t stateEnteredMs = 0U;
    bool returnLevelPending = false;
    /* This centre is only used for guarded pre-reference manual adjustment.
     * It starts at raw lower-stop POS 0 and becomes -565 after the PB0 lift. */
    int32_t preReferenceCenterPos = LOWER_STOP_POS_STEPS;
#endif

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
#if BALL_BEAM_TASK4_BASELINE
    /* Motor_init resets GPIOA/B, so it must precede every GPIO client.  It
     * finishes with both TB6612 bridges released. */
    Motor_init();
#endif
    button_init();
    startButton.stable = button_pressed() & BUTTON_START_PIN;
    startButton.candidate = startButton.stable;
    upButton.stable = button_pressed() & BUTTON_UP_PIN;
    upButton.candidate = upButton.stable;
    downButton.stable = button_pressed() & BUTTON_DOWN_PIN;
    downButton.candidate = downButton.stable;
    presetLiftButton.stable = button_pressed() & BUTTON_PRESET_LIFT_PIN;
    presetLiftButton.candidate = presetLiftButton.stable;
    levelZeroButton.stable = button_pressed() & BUTTON_LEVEL_ZERO_PIN;
    levelZeroButton.candidate = levelZeroButton.stable;
    StepperBeam_init();
    /* Keep the active-low X42S enable inactive at power-up.  The first
     * deliberate START press arms the guarded level-calibration state. */
    StepperBeam_setArmed(false);
#if BALL_BEAM_TASK4_BASELINE
    Encoder_init();
    Encoder_resetCounts();
    Task4Baseline_init(&task4Profile, NULL);
    imuReady = Icm42688_init();
#endif
    VisionUart_init();
    DiagnosticUart_init();
    BallBeamController_init(&controller, BALL_TARGET_X_PIXEL);
    (void) BeamCalibration_angleToPos(0, &command.calibration);
    wait_ms(BOOT_SETTLE_MS);
    (void) Oled_init();

    while (1) {
        const uint32_t heldButtons = button_pressed();
        const uint32_t pressed = button_update(
            &startButton, BUTTON_START_PIN, heldButtons);
#if BALL_BEAM_TASK3_SEQUENCE
        const uint32_t upPressed = button_update(
            &upButton, BUTTON_UP_PIN, heldButtons);
        const uint32_t downPressed = button_update(
            &downButton, BUTTON_DOWN_PIN, heldButtons);
#endif
#if !BALL_BEAM_TASK3_SEQUENCE
        const uint32_t presetLiftPressed = button_update(
            &presetLiftButton, BUTTON_PRESET_LIFT_PIN, heldButtons);
        const uint32_t levelZeroPressed = button_update(
            &levelZeroButton, BUTTON_LEVEL_ZERO_PIN, heldButtons);
#endif
        const bool gotFrame = VisionUart_pollNewest(&vision);
        bool linkFresh;

        StepperBeam_service();
        StepperBeam_getStatus(&motor);

        if (gotFrame) {
            haveReceivedPacket = true;
            lastRxMs = nowMs;
            g_ball_last_packet_ms = nowMs;
            g_ball_x_offset_px = vision.xOffsetPx;
            g_ball_confidence_milli = vision.confidenceMilli;
            ballIsValid =
                (vision.lost == 0U) &&
                (vision.confidenceMilli >= VISION_MIN_CONFIDENCE_MILLI);
#if BALL_BEAM_TASK4_BASELINE
            if (ballIsValid) {
                haveValidBall = true;
                lastValidBallMs = nowMs;
            }
#endif
        }
        linkFresh = haveReceivedPacket &&
                    ((nowMs - lastRxMs) <= VISION_PACKET_TIMEOUT_MS);
#if BALL_BEAM_TASK4_BASELINE
        const bool task4BallFresh = haveValidBall &&
            ((nowMs - lastValidBallMs) <= VISION_PACKET_TIMEOUT_MS);
        if (linkFresh && ballIsValid && task4BallFresh &&
            (vision.xOffsetPx >= -TASK4_START_MAX_ERROR_PX) &&
            (vision.xOffsetPx <= TASK4_START_MAX_ERROR_PX)) {
            if (!task4CenterStable) {
                task4CenterStable = true;
                task4CenterStableSinceMs = nowMs;
            }
        } else {
            task4CenterStable = false;
            task4CenterStableSinceMs = nowMs;
        }
#endif

#if !BALL_BEAM_TASK3_SEQUENCE
        if ((presetLiftPressed & BUTTON_PRESET_LIFT_PIN) != 0U) {
            /* PB0 is an explicit operator confirmation that the mechanism
             * was physically parked at its lower stop before reset.  There
             * is no limit switch, so do not infer that fact from position. */
            if ((closedLoopState == CLOSED_LOOP_WAIT_LEVEL) && !levelSaved &&
                (motor.moving == 0U) &&
                (motor.positionSteps == LOWER_STOP_POS_STEPS) &&
                (motor.travelLimitsConfigured == 0U) &&
                StepperBeam_queueBoundedJog(
                    LOWER_STOP_TO_LEVEL_STEPS,
                    LOWER_STOP_TO_LEVEL_STEPS,
                    LOWER_STOP_POS_STEPS)) {
                closedLoopState = CLOSED_LOOP_PRESET_LIFT;
                stateEnteredMs = nowMs;
                message = "PB0 LIFT TO LEVEL";
            } else if ((closedLoopState == CLOSED_LOOP_WAIT_LEVEL) &&
                       levelSaved) {
                /* START stops asynchronously.  PB0 immediately afterwards
                 * must wait for the last pulse to retire, rather than turn a
                 * valid return-to-level request into a refusal. */
                if (motor.moving != 0U) {
                    StepperBeam_stop();
                    StepperBeam_setArmed(true);
                    returnLevelPending = true;
                    closedLoopState = CLOSED_LOOP_JOG_STOP;
                    stateEnteredMs = nowMs;
                    message = "PB0 WAIT STOP";
                } else if (StepperBeam_setTargetSteps(levelReferencePos)) {
                    /* The reference and travel limits are already validated;
                     * command only the saved absolute level coordinate. */
                    reset_control_history(&controller, &command);
                    closedLoopState = CLOSED_LOOP_RETURN_LEVEL;
                    stateEnteredMs = nowMs;
                    message = "PB0 RETURN LEVEL";
                } else {
                    closedLoopState = target_rejection_state(
                        &motor, levelReferencePos);
                    stateEnteredMs = nowMs;
                    message = "PB0 RETURN REFUSED";
                }
            } else {
                /* PB0 must never coexist with jog or automatic tracking. A
                 * misuse is a stop request, not a way to retain motion. */
                StepperBeam_stop();
                StepperBeam_setArmed(true);
                if (((closedLoopState == CLOSED_LOOP_ACTIVE) ||
                     (closedLoopState == CLOSED_LOOP_ZERO_ACCEPTED)) &&
                    levelSaved) {
                    /* PB0 is allowed to follow START immediately.  Keep the
                     * accepted reference, stop first, then return to it once
                     * the pulse engine is idle. */
                    reset_control_history(&controller, &command);
                    returnLevelPending = true;
                    closedLoopState = CLOSED_LOOP_JOG_STOP;
                    message = "PB0 WAIT STOP";
                } else {
                    closedLoopState = CLOSED_LOOP_PRESET_REFUSED;
                    message = "PB0 REFUSED STOP";
                }
                stateEnteredMs = nowMs;
            }
            StepperBeam_getStatus(&motor);
        }

        if ((levelZeroPressed & BUTTON_LEVEL_ZERO_PIN) != 0U) {
            /* PB20 is the one-shot operator acceptance of the manually
             * leveled beam.  It deliberately does not enable tracking. */
            if ((closedLoopState == CLOSED_LOOP_WAIT_LEVEL) && !levelSaved &&
                (motor.moving == 0U) && save_level_reference(motor.positionSteps)) {
                levelReferencePos = motor.positionSteps;
                levelSaved = true;
                preReferenceCenterPos = levelReferencePos;
                reset_control_history(&controller, &command);
                closedLoopState = CLOSED_LOOP_ZERO_ACCEPTED;
                stateEnteredMs = nowMs;
                message = "PB20 ZERO ACCEPTED";
            } else {
                StepperBeam_stop();
                StepperBeam_setArmed(true);
                closedLoopState = CLOSED_LOOP_PRESET_REFUSED;
                stateEnteredMs = nowMs;
                message = "PB20 ZERO REFUSED";
            }
            StepperBeam_getStatus(&motor);
        }
#endif

        if ((pressed & BUTTON_START_PIN) != 0U) {
            if (motor.armed != 0U) {
#if BALL_BEAM_TASK3_SEQUENCE
                if (task3Phase == TASK3_LEVEL_CAL) {
                    if (!lowerStopLiftDone) {
                        /* There is no physical limit switch: raw coordinate
                         * zero is valid only after power-up at the manually
                         * parked lower stop.  Refuse an ambiguous move. */
                        if (motor.positionSteps != 0) {
                            message = "PARK LOW REBOOT";
                        } else if (StepperBeam_queueJog(
                                       LOWER_STOP_TO_LEVEL_STEPS)) {
                            lowerStopLiftDone = true;
                            message = "LIFT TO POS-565";
                        } else {
                            message = "LIFT START FAILED";
                        }
                    } else if (motor.moving != 0U) {
                        message = "LEVEL CAL WAIT";
                    } else if (save_level_reference(motor.positionSteps)) {
                        levelReferencePos = motor.positionSteps;
                        levelSaved = true;
                        task3Phase = TASK3_READY;
                        (void) BeamCalibration_angleToPos(0,
                                                          &command.calibration);
                        command.requestedAngleMdeg = 0;
                        message = "LEVEL SAVED START";
                    } else {
                        message = "LEVEL SAVE FAILED";
                    }
                } else if (task3Phase == TASK3_READY) {
                    task3Phase = TASK3_TO_PLUS5;
                    task3StartedMs = nowMs;
                    task3_set_target(&controller, TASK3_TARGET_PLUS5_PX);
                    message = "TASK3 GO +5CM";
                } else {
                    /* A new START returns to the saved physical level. */
                    task3Phase = TASK3_READY;
                    (void) StepperBeam_setTargetSteps(levelReferencePos);
                    (void) BeamCalibration_angleToPos(0, &command.calibration);
                    command.requestedAngleMdeg = 0;
                    message = "RETURN LEVEL";
                }
#else
                if (closedLoopState == CLOSED_LOOP_LIMIT_FAULT) {
                    if (StepperBeam_clearFault()) {
                        StepperBeam_setArmed(true);
                        reset_control_history(&controller, &command);
                        closedLoopState = CLOSED_LOOP_WAIT_LEVEL;
                        stateEnteredMs = nowMs;
                        message = "FAULT CLEARED LEVEL";
                    }
                } else if (closedLoopState == CLOSED_LOOP_PRESET_REFUSED) {
                    /* A rejected PB0/PB20 request is never a shortcut back
                     * into tracking. START only acknowledges the stopped
                     * condition and returns the operator to the safe state. */
                    stop_for_wait_level();
                    reset_control_history(&controller, &command);
                    closedLoopState = CLOSED_LOOP_WAIT_LEVEL;
                    stateEnteredMs = nowMs;
                    message = "REFUSED -> WAIT_LEVEL";
                } else if ((closedLoopState == CLOSED_LOOP_WAIT_LEVEL) ||
                           (closedLoopState == CLOSED_LOOP_JOG_STOP) ||
                           (closedLoopState == CLOSED_LOOP_LEVEL_READY)) {
                    if (motor.moving != 0U) {
                        StepperBeam_stop();
                        StepperBeam_setArmed(true);
                        closedLoopState = CLOSED_LOOP_JOG_STOP;
                        stateEnteredMs = nowMs;
                        message = "JOG STOP THEN START";
                    } else if (levelSaved) {
                        reset_control_history(&controller, &command);
#if BALL_BEAM_TASK4_BASELINE
                        if (!imuReady || !linkFresh || !ballIsValid ||
                            !task4CenterStable ||
                            ((nowMs - task4CenterStableSinceMs) <
                             TASK4_START_CENTER_DWELL_MS)) {
                            closedLoopState = CLOSED_LOOP_LEVEL_READY;
                            stateEnteredMs = nowMs;
                            Task4Baseline_stop(&task4Profile);
                            Motor_stopAll();
                            message = !imuReady ? "IMU FAULT NO DRIVE" :
                                (!linkFresh || !ballIsValid) ?
                                "VISION NOT READY" : "CENTER BALL 500MS";
                        } else {
                            closedLoopState = CLOSED_LOOP_ACTIVE;
                            stateEnteredMs = nowMs;
                            Task4Baseline_arm(&task4Profile);
                            Encoder_resetCounts();
                            lastLeftEncoder = 0;
                            lastRightEncoder = 0;
                            (void) Task4Baseline_start(&task4Profile, nowMs);
                            message = "TASK4 BASELINE RUN";
                        }
#else
                        closedLoopState = CLOSED_LOOP_ACTIVE;
                        stateEnteredMs = nowMs;
                        message = "ACTIVE START";
#endif
                    } else {
                        closedLoopState = CLOSED_LOOP_TARGET_REJECTED_UNREFERENCED;
                        stateEnteredMs = nowMs;
                        message = "PB20 SAVE ZERO";
                    }
                } else {
                    /* START is an immediate operator stop in every automatic
                     * state.  The PB20 reference remains valid so PB0 can
                     * return to it after the test. */
                    stop_for_wait_level();
#if BALL_BEAM_TASK4_BASELINE
                    Task4Baseline_stop(&task4Profile);
                    Motor_stopAll();
#endif
                    reset_control_history(&controller, &command);
                    closedLoopState = CLOSED_LOOP_WAIT_LEVEL;
                    stateEnteredMs = nowMs;
                    message = "START -> WAIT_LEVEL";
                }
#endif
            } else {
                StepperBeam_setArmed(true);
#if BALL_BEAM_TASK3_SEQUENCE
                message = "LEVEL CAL UP/DN";
#else
                StepperBeam_setArmed(true);
                reset_control_history(&controller, &command);
                closedLoopState = CLOSED_LOOP_WAIT_LEVEL;
                stateEnteredMs = nowMs;
                message = "WAIT_LEVEL";
#endif
            }
            StepperBeam_getStatus(&motor);
        }

        if (motor.armed != 0U) {
#if BALL_BEAM_TASK3_SEQUENCE
            if (task3Phase == TASK3_LEVEL_CAL) {
                const uint32_t held = button_pressed();
                int16_t jog = 0;

                if (lowerStopLiftDone && (motor.moving != 0U)) {
                    message = "LIFT TO POS-565";
                } else if (!lowerStopLiftDone) {
                    message = "PARK LOW START";
                } else if (((upPressed & BUTTON_UP_PIN) != 0U) ||
                    (((held & BUTTON_UP_PIN) != 0U) &&
                     (nowMs >= nextLevelJogMs))) {
                    jog = LEVEL_JOG_STEPS;
                } else if (((downPressed & BUTTON_DOWN_PIN) != 0U) ||
                           (((held & BUTTON_DOWN_PIN) != 0U) &&
                            (nowMs >= nextLevelJogMs))) {
                    jog = -LEVEL_JOG_STEPS;
                }
                if ((jog != 0) && StepperBeam_queueJog(jog)) {
                    nextLevelJogMs = nowMs + LEVEL_HOLD_REPEAT_MS;
                    message = jog > 0 ? "LEVEL CAL UP" : "LEVEL CAL DOWN";
                } else if (motor.moving == 0U) {
                    message = lowerStopLiftDone ? "LIFT DONE START OK" :
                                                   "PARK LOW START";
                }
            } else if ((task3Phase == TASK3_READY) ||
                       (task3Phase == TASK3_TIMEOUT)) {
                (void) StepperBeam_setTargetSteps(levelReferencePos);
                (void) BeamCalibration_angleToPos(0, &command.calibration);
                command.requestedAngleMdeg = 0;
                g_ball_requested_beam_angle_mdeg = 0;
                g_ball_target_pos = levelReferencePos;
                message = (task3Phase == TASK3_READY) ?
                    "LEVEL READY START" : "TASK3 TIMEOUT LVL";
            } else if (((task3Phase == TASK3_TO_PLUS5) ||
                        (task3Phase == TASK3_TO_MINUS5) ||
                        (task3Phase == TASK3_HOLD_MINUS5)) &&
                       gotFrame && ballIsValid && linkFresh) {
#else
            if ((closedLoopState == CLOSED_LOOP_ACTIVE) &&
                gotFrame && ballIsValid && linkFresh) {
#endif
                bool commandReady = BallBeamController_update(
                    &controller,
                    vision.xOffsetPx,
                    nowMs,
                    BALL_CONTROL_SIGN,
                    &command);
#if BALL_BEAM_TASK3_SEQUENCE
                const bool reverseBrakeTriggered =
                    commandReady && (task3Phase == TASK3_TO_MINUS5) &&
                    task3_predicts_minus5_overshoot(&command);
                if (commandReady) {
                    commandReady = task3_limit_command(
                        &command, task3Phase, vision.xOffsetPx);
                }
#endif
                if (commandReady &&
#if BALL_BEAM_TASK3_SEQUENCE
                    set_relative_control_target(
                        &command.calibration, levelReferencePos)
#else
                    set_relative_control_target(
                        &command.calibration, levelReferencePos)
#endif
                    ) {
                    g_ball_requested_beam_angle_mdeg =
                        command.requestedAngleMdeg;
#if BALL_BEAM_TASK3_SEQUENCE
                    g_ball_target_pos = levelReferencePos +
                                        command.calibration.targetPos;
#else
                    g_ball_target_pos = levelReferencePos +
                                        command.calibration.targetPos;
#endif
                    g_ball_cal_angle_low_mdeg =
                        command.calibration.angleLowMdeg;
                    g_ball_cal_angle_high_mdeg =
                        command.calibration.angleHighMdeg;
                    g_ball_cal_pos_low = command.calibration.posLow;
                    g_ball_cal_pos_high = command.calibration.posHigh;
#if BALL_BEAM_TASK3_SEQUENCE
                    if (task3Phase == TASK3_TO_PLUS5) {
                        /*
                         * The task accepts +/-1 cm at +5 cm.  Do not wait
                         * for an exact crossing of +61 px: the +side brake
                         * may settle at e.g. +4.5 cm, which is already a
                         * valid arrival but would otherwise never reverse.
                         */
                        if (vision.xOffsetPx >=
                            TASK3_POSITIVE_REVERSE_TRIGGER_PX) {
                            task3Phase = TASK3_TO_MINUS5;
                            task3_change_target(
                                &controller, TASK3_TARGET_MINUS5_PX);
                            message = "TASK3 REV -5CM";
                        } else {
                            message = "TASK3 GO +5CM";
                        }
                    } else if (task3Phase == TASK3_TO_MINUS5) {
                        if (reverseBrakeTriggered) {
                            /* This frame has sent the calibrated reverse-brake
                             * command.  Hold that tilt on following frames
                             * until the measured endpoint is actually valid. */
                            task3Phase = TASK3_HOLD_MINUS5;
                            message = "TASK3 BRAKE -5";
                        } else
                        /* User-selected endpoint rule: entering the -5 cm
                         * calibrated tolerance band ends the task immediately.
                         * Match the normal timeout behaviour by returning the
                         * beam to its saved physical level reference. */
                        if (absolute_error(vision.xOffsetPx,
                                           TASK3_TARGET_MINUS5_PX) <=
                            TASK3_TOLERANCE_MINUS5_PX) {
                            (void) StepperBeam_setTargetSteps(levelReferencePos);
                            (void) BeamCalibration_angleToPos(
                                0, &command.calibration);
                            command.requestedAngleMdeg = 0;
                            g_ball_requested_beam_angle_mdeg = 0;
                            g_ball_target_pos = levelReferencePos;
                            task3Phase = TASK3_TIMEOUT;
                            message = "TASK3 -5 STOP";
                        } else {
                            message = "TASK3 GO -5CM";
                        }
                    } else if (task3Phase == TASK3_HOLD_MINUS5) {
                        if (absolute_error(vision.xOffsetPx,
                                           TASK3_TARGET_MINUS5_PX) <=
                            TASK3_TOLERANCE_MINUS5_PX) {
                            (void) StepperBeam_setTargetSteps(levelReferencePos);
                            (void) BeamCalibration_angleToPos(
                                0, &command.calibration);
                            command.requestedAngleMdeg = 0;
                            g_ball_requested_beam_angle_mdeg = 0;
                            g_ball_target_pos = levelReferencePos;
                            task3Phase = TASK3_TIMEOUT;
                            message = "TASK3 -5 STOP";
                        } else {
                            message = "TASK3 BRAKE -5";
                        }
                    }
#else
                    message = "ACTIVE";
#endif
                } else {
#if !BALL_BEAM_TASK3_SEQUENCE
                    StepperBeam_getStatus(&motor);
                    closedLoopState = target_rejection_state(
                        &motor, levelReferencePos +
                        command.calibration.targetPos);
                    stateEnteredMs = nowMs;
                    StepperBeam_stop();
                    StepperBeam_setArmed(true);
#if BALL_BEAM_TASK4_BASELINE
                    Task4Baseline_stop(&task4Profile);
                    Motor_stopAll();
#endif
#endif
                    message = "CMD REFUSED";
                }
            }
#if BALL_BEAM_TASK3_SEQUENCE
            else if (((task3Phase == TASK3_TO_PLUS5) ||
                      (task3Phase == TASK3_TO_MINUS5) ||
                      (task3Phase == TASK3_HOLD_MINUS5)) &&
                     (!linkFresh || (gotFrame && !ballIsValid))) {
                /*
                 * Never keep a stale camera-derived tilt. Level the beam by
                 * commanding the calibrated POS 0, but stay armed so holding
                 * torque keeps the mechanism at the safe reference.
                 */
                (void) StepperBeam_setTargetSteps(levelReferencePos);
                (void) BeamCalibration_angleToPos(0, &command.calibration);
                command.requestedAngleMdeg = 0;
                g_ball_requested_beam_angle_mdeg = 0;
                g_ball_target_pos = levelReferencePos;
                message = linkFresh ? "NO BALL -> LEVEL" :
                                      "TIMEOUT -> LEVEL";
#else
            else if ((closedLoopState == CLOSED_LOOP_ACTIVE) &&
#if BALL_BEAM_TASK4_BASELINE
                     (!linkFresh || !task4BallFresh)) {
#else
                     (!linkFresh || (gotFrame && !ballIsValid))) {
#endif
                StepperBeam_stop();
                StepperBeam_setArmed(true);
#if BALL_BEAM_TASK4_BASELINE
                Task4Baseline_stop(&task4Profile);
                Motor_stopAll();
#endif
                reset_control_history(&controller, &command);
                g_ball_requested_beam_angle_mdeg = 0;
                g_ball_target_pos = motor.positionSteps;
                closedLoopState = CLOSED_LOOP_LOST;
                stateEnteredMs = nowMs;
                message = linkFresh ? "LOST BALL STOP" : "LOST LINK STOP";
#endif
            }
        }

#if !BALL_BEAM_TASK3_SEQUENCE
        StepperBeam_getStatus(&motor);
        if ((closedLoopState == CLOSED_LOOP_ACTIVE) &&
            (motor.travelLimitsConfigured != 0U) &&
            (((motor.positionSteps <= motor.travelMinSteps) &&
              (motor.targetSteps <= motor.travelMinSteps)) ||
             ((motor.positionSteps >= motor.travelMaxSteps) &&
              (motor.targetSteps >= motor.travelMaxSteps)))) {
            /* The normal tracker can only reach a boundary through an
             * accepted in-range target.  Treat arrival at either boundary as
             * a terminal safe event, never as a point to keep driving. */
            StepperBeam_stop();
#if BALL_BEAM_TASK4_BASELINE
            Task4Baseline_stop(&task4Profile);
            Motor_stopAll();
#endif
            closedLoopState = CLOSED_LOOP_LIMIT_FAULT;
            stateEnteredMs = nowMs;
            reset_control_history(&controller, &command);
            message = "TRAVEL LIMIT STOP";
        } else if (motor.fault != STEPPER_BEAM_FAULT_NONE) {
            StepperBeam_stop();
#if BALL_BEAM_TASK4_BASELINE
            Task4Baseline_stop(&task4Profile);
            Motor_stopAll();
#endif
            closedLoopState = CLOSED_LOOP_LIMIT_FAULT;
            stateEnteredMs = nowMs;
            reset_control_history(&controller, &command);
            message = "LIMIT FAULT STOP";
        } else if (closedLoopState == CLOSED_LOOP_ZERO_ACCEPTED) {
            /* PB20 records its acceptance before the explicitly separate
             * START request can enable automatic control. */
            if ((nowMs - stateEnteredMs) >= DIAGNOSTIC_LOG_PERIOD_MS) {
                reset_control_history(&controller, &command);
                closedLoopState = CLOSED_LOOP_LEVEL_READY;
                stateEnteredMs = nowMs;
                message = "LEVEL READY START";
            }
        } else if (closedLoopState == CLOSED_LOOP_PRESET_LIFT) {
            if (motor.moving == 0U) {
                /* The finite bounded jog has completed at the old, measured
                 * level approximation.  START is still required to accept
                 * it as the actual level zero. */
                preReferenceCenterPos = motor.positionSteps;
                closedLoopState = CLOSED_LOOP_WAIT_LEVEL;
                stateEnteredMs = nowMs;
                message = "PB0 LIFT DONE START";
            }
        } else if (closedLoopState == CLOSED_LOOP_RETURN_LEVEL) {
            if (motor.moving == 0U) {
                reset_control_history(&controller, &command);
                closedLoopState = CLOSED_LOOP_WAIT_LEVEL;
                stateEnteredMs = nowMs;
                message = "PB0 LEVEL DONE";
            }
        } else if (returnLevelPending) {
            /* A PB0 request received during START's final stop completes
             * only after the pulse engine reports idle. */
            if (motor.moving == 0U) {
                if (StepperBeam_setTargetSteps(levelReferencePos)) {
                    returnLevelPending = false;
                    reset_control_history(&controller, &command);
                    closedLoopState = CLOSED_LOOP_RETURN_LEVEL;
                    stateEnteredMs = nowMs;
                    message = "PB0 RETURN LEVEL";
                } else {
                    returnLevelPending = false;
                    closedLoopState = target_rejection_state(
                        &motor, levelReferencePos);
                    stateEnteredMs = nowMs;
                    message = "PB0 RETURN REFUSED";
                }
            }
        } else if ((closedLoopState == CLOSED_LOOP_WAIT_LEVEL) ||
                   (closedLoopState == CLOSED_LOOP_JOG_LEFT) ||
                   (closedLoopState == CLOSED_LOOP_JOG_RIGHT) ||
                   (closedLoopState == CLOSED_LOOP_JOG_STOP)) {
            const uint32_t held = button_pressed();
            const bool jogRight = (held & BUTTON_UP_PIN) != 0U;
            const bool jogLeft = (held & BUTTON_DOWN_PIN) != 0U;

            if (jogRight && !jogLeft) {
                if (StepperBeam_startManualJog(
                        1, LEVEL_MANUAL_JOG_PPS,
                        preReferenceCenterPos + WAIT_LEVEL_JOG_MIN_STEPS,
                        preReferenceCenterPos + WAIT_LEVEL_JOG_MAX_STEPS)) {
                    closedLoopState = CLOSED_LOOP_JOG_RIGHT;
                    message = "JOG RIGHT HOLD";
                } else {
                    StepperBeam_getStatus(&motor);
                    closedLoopState = target_rejection_state(
                        &motor, motor.positionSteps);
                    stateEnteredMs = nowMs;
                    message = "JOG REFUSED";
                }
            } else if (jogLeft && !jogRight) {
                if (StepperBeam_startManualJog(
                        -1, LEVEL_MANUAL_JOG_PPS,
                        preReferenceCenterPos + WAIT_LEVEL_JOG_MIN_STEPS,
                        preReferenceCenterPos + WAIT_LEVEL_JOG_MAX_STEPS)) {
                    closedLoopState = CLOSED_LOOP_JOG_LEFT;
                    message = "JOG LEFT HOLD";
                } else {
                    StepperBeam_getStatus(&motor);
                    closedLoopState = target_rejection_state(
                        &motor, motor.positionSteps);
                    stateEnteredMs = nowMs;
                    message = "JOG REFUSED";
                }
            } else if ((closedLoopState == CLOSED_LOOP_JOG_LEFT) ||
                       (closedLoopState == CLOSED_LOOP_JOG_RIGHT)) {
                /* Releasing either key, or pressing both, stops STEP pulses
                 * immediately.  A direction reversal always goes through
                 * this stopped state. */
                StepperBeam_stop();
                StepperBeam_setArmed(true);
                closedLoopState = CLOSED_LOOP_JOG_STOP;
                stateEnteredMs = nowMs;
                message = "JOG STOP";
            } else if ((closedLoopState == CLOSED_LOOP_JOG_STOP) &&
                       ((nowMs - stateEnteredMs) >= DIAGNOSTIC_LOG_PERIOD_MS)) {
                closedLoopState = CLOSED_LOOP_WAIT_LEVEL;
                stateEnteredMs = nowMs;
                message = "WAIT LEVEL";
            }
        }
#endif

#if BALL_BEAM_TASK3_SEQUENCE
        if ((motor.armed != 0U) &&
            ((task3Phase == TASK3_TO_PLUS5) ||
             (task3Phase == TASK3_TO_MINUS5) ||
             (task3Phase == TASK3_HOLD_MINUS5)) &&
            ((nowMs - task3StartedMs) >= TASK3_TIMEOUT_MS)) {
            (void) StepperBeam_setTargetSteps(levelReferencePos);
            task3Phase = TASK3_TIMEOUT;
            message = "TASK3 TIMEOUT LVL";
        }
#endif

#if BALL_BEAM_TASK4_BASELINE
        if (!imuReady && (closedLoopState != CLOSED_LOOP_ACTIVE) &&
            ((nowMs - lastImuRetryMs) >= TASK4_IMU_RETRY_PERIOD_MS)) {
            imuReady = Icm42688_init();
            imuSampleValid = false;
            lastImuRetryMs = nowMs;
        }
        if ((nowMs - lastImuSampleMs) >= TASK4_IMU_SAMPLE_PERIOD_MS) {
            imuSampleValid = imuReady && Icm42688_readSample(&imuSample);
            lastImuSampleMs = nowMs;
            if (imuSampleValid) {
                haveValidImu = true;
                lastValidImuMs = nowMs;
            } else {
                Motor_stopAll();
                if ((closedLoopState == CLOSED_LOOP_ACTIVE) &&
                    (!haveValidImu ||
                     ((nowMs - lastValidImuMs) > VISION_PACKET_TIMEOUT_MS))) {
                    imuReady = false;
                    Task4Baseline_stop(&task4Profile);
                    StepperBeam_stop();
                    StepperBeam_setArmed(true);
                    reset_control_history(&controller, &command);
                    closedLoopState = CLOSED_LOOP_LOST;
                    stateEnteredMs = nowMs;
                    message = "IMU SAMPLE FAULT";
                } else if (closedLoopState != CLOSED_LOOP_ACTIVE) {
                    imuReady = false;
                    Task4Baseline_stop(&task4Profile);
                }
            }
        }
        if ((closedLoopState == CLOSED_LOOP_ACTIVE) && imuSampleValid &&
            linkFresh && task4BallFresh) {
            task4Drive = Task4Baseline_step(&task4Profile, nowMs);
            if (task4Drive.driveEnabled) {
                Motor_setBothDuty(MOTOR_DIRECTION_FORWARD,
                                  MOTOR_DIRECTION_FORWARD,
                                  task4Drive.dutyPercent,
                                  task4Drive.dutyPercent);
            } else {
                Motor_stopAll();
            }
            if (task4Drive.justStopped) {
                message = "TASK4 PROFILE STOP";
            }
        } else {
            Motor_stopAll();
        }
#endif

        StepperBeam_getStatus(&motor);
        if ((nowMs - lastDiagnosticLogMs) >= DIAGNOSTIC_LOG_PERIOD_MS) {
            const int32_t ageMs = haveReceivedPacket ?
                (int32_t) (nowMs - lastRxMs) : -1;
#if BALL_BEAM_TASK4_BASELINE
            const int32_t leftEncoder = g_motorA_encoder_count;
            const int32_t rightEncoder = g_motorB_encoder_count;
            const uint32_t speedDtMs = nowMs - lastDiagnosticLogMs;
            const int32_t ballErrorPx = vision.xOffsetPx -
                                        controller.targetXOffsetPx;
            const int32_t ballErrorCmX100 = (ballErrorPx >= 0) ?
                (ballErrorPx * 1000) / 118 : (ballErrorPx * 100) / 13;
            if (speedDtMs != 0U) {
                leftSpeedCountsPerS =
                    ((leftEncoder - lastLeftEncoder) * 1000) /
                    (int32_t) speedDtMs;
                rightSpeedCountsPerS =
                    ((rightEncoder - lastRightEncoder) * 1000) /
                    (int32_t) speedDtMs;
            }
            DiagnosticUart_writeTask4Csv(
                nowMs, &vision, ballErrorCmX100, ageMs,
                command.velocityPxPerS, command.requestedAngleMdeg, &motor,
                leftEncoder, rightEncoder, leftSpeedCountsPerS,
                rightSpeedCountsPerS,
                imuSampleValid ? imuSample.accelX : TASK4_IMU_MISSING_RAW,
                imuSampleValid ? imuSample.accelY : TASK4_IMU_MISSING_RAW,
                imuSampleValid ? imuSample.accelZ : TASK4_IMU_MISSING_RAW,
                g_motor_pwm_a_duty, g_motor_pwm_b_duty,
                Task4Baseline_phaseName(task4Profile.phase),
                imuReady ? (imuSampleValid ?
                    closed_loop_state_name(closedLoopState) : "IMU_READ_FAULT") :
                    "IMU_INIT_FAULT");
            lastLeftEncoder = leftEncoder;
            lastRightEncoder = rightEncoder;
#else
            DiagnosticUart_writeCsv(
                nowMs, &vision, ageMs, controller.targetXOffsetPx,
                command.velocityPxPerS, command.errorPx,
                command.requestedAngleMdeg, &motor,
#if BALL_BEAM_TASK3_SEQUENCE
                diagnostic_state(&motor, linkFresh, ballIsValid, &command));
#else
                closed_loop_state_name(closedLoopState));
#endif
#endif
            lastDiagnosticLogMs = nowMs;
        }
        update_oled(
            &motor, &vision, &command, linkFresh,
#if BALL_BEAM_TASK3_SEQUENCE
            levelReferencePos, levelSaved,
#else
            levelReferencePos, levelSaved,
#endif
            message, nowMs);
        wait_ms(BUTTON_TICK_MS);
        nowMs += BUTTON_TICK_MS;
    }
}
