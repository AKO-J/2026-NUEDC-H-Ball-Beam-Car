#include <stdbool.h>
#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "beam_calibration.h"
#include "ball_beam_controller.h"
#include "diagnostic_uart.h"
#include "encoder.h"
#include "h_track_controller.h"
#include "h_track_pwm_config.h"
#include "icm42688.h"
#include "ir_line_sensor.h"
#include "joint_line_balance_control.h"
#include "motor_driver.h"
#include "ssd1306_oled.h"
#include "stepper_beam.h"
#include "vision_uart.h"

/* Independent, conservative straight-line joint bring-up image.
 * Every numeric tuning value below is UNVERIFIED until recorded bench tests
 * freeze it. No parameter is modified automatically at runtime. */
#define TICK_MS                         5U
#define SPEED_PERIOD_MS                20U
#define IMU_PERIOD_MS                  10U
#define LOG_PERIOD_MS                  50U
#define MCLK_CYCLES_PER_MS          32000U
#define START_PORT                    GPIOB
#define START_PIN                     DL_GPIO_PIN_7
#define START_IOMUX                   IOMUX_PINCM24
#define DOWN_PIN                      DL_GPIO_PIN_8
#define DOWN_IOMUX                    IOMUX_PINCM25
#define UP_PIN                        DL_GPIO_PIN_15
#define UP_IOMUX                      IOMUX_PINCM32
#define VISION_RED_LED_PORT           GPIOA
#define VISION_RED_LED_PIN            DL_GPIO_PIN_0
#define VISION_RED_LED_IOMUX          IOMUX_PINCM1
#define VISION_GREEN_LED_PORT         GPIOB
#define VISION_GREEN_LED_PIN          DL_GPIO_PIN_27
#define VISION_GREEN_LED_IOMUX        IOMUX_PINCM58
#define BUTTON_DEBOUNCE_MS             30U
#define LOWER_STOP_TO_LEVEL_STEPS    (-565)
#define LOWER_STOP_POS_STEPS             0
#define LEVEL_MANUAL_JOG_PPS            80U
#define VISION_TIMEOUT_MS             150U
#define VISION_MIN_CONFIDENCE         320U
#define BALL_LIMIT_CM_X100            100
#define BALL_CENTER_DWELL_MS          500U
#define LINE_LOST_HOLD_MS              80U
#define LEFT_COUNTS_PER_100_CM        6445L
#define RIGHT_COUNTS_PER_100_CM       6267L
#define MAX_SPEED_X100                800L /* 8 cm/s, UNVERIFIED ceiling. */
#define CRUISE_SPEED_X100             500L /* 5 cm/s, UNVERIFIED candidate. */
#define ACCEL_LIMIT_X100              100L /* Keep verified gentle PWM ramp. */
#define JERK_LIMIT_X100                60L /* 0.60 cm/s^3: softer onset. */
#define KFF_MILLI                    1000  /* 1.00 * atan(a/g) feed-forward. */
#define KFF_SIGN                        1  /* MUST be checked without ball. */
#define STARTUP_PWM_RAMP_MS            750U
#define STARTUP_PRETILT_HOLD_MS        250U
#define STARTUP_FEEDFORWARD_BEGIN_MS   600U
#define STARTUP_LEFT_BELOW_DUTY          4U /* Measured breakaway is about 5%. */
#define STARTUP_RIGHT_BELOW_DUTY         5U /* Measured breakaway is about 6%. */
#define STARTUP_LEFT_RUN_DUTY            5U
#define STARTUP_RIGHT_RUN_DUTY           6U
#define STARTUP_PRETILT_MDEG             60
#define BALL_CONTROL_SIGN               1  /* Verified by task 3. */
#define JOINT_BALL_KP_MDEG_PER_PX       11  /* Static stable value; 12/14 oscillated. */
#define BALL_PD_LIMIT_MDEG             400
#define IMU_MISSING          (-2147483647L - 1L)

enum {
    FAULT_NONE = 0U,
    FAULT_VISION_LOST = 1U << 0,
    FAULT_BALL_LIMIT = 1U << 1,
    FAULT_LINE_LOST = 1U << 2,
    FAULT_ENCODER_STALL = 1U << 3,
    FAULT_STEPPER = 1U << 4,
    FAULT_EMERGENCY = 1U << 5,
};

typedef enum {
    JOINT_WAIT_LEVEL = 0, JOINT_READY, JOINT_RAMP_UP, JOINT_CRUISE,
    JOINT_FAULT
} JointState;

typedef enum {
    LEVEL_LOCKED = 0, LEVEL_CONFIRM_LOWER_STOP, LEVEL_PRESET_LIFT,
    LEVEL_FINE_ADJUST, LEVEL_SAVED
} JointLevelStage;

/* Line-first test mode: error=+/-200 now requests +/-50 speed-x100 of
 * differential, which maps to roughly 4 PWM points between the wheels at
 * the current 22% ceiling. Kd and all wheel settings remain unchanged. */
static const JointLinePdConfig k_line = {25, 1, 64, 250, 20, LINE_LOST_HOLD_MS};
static const HTrackConfig k_task2_line = {
    H_TRACK_LF04_CENTER_LEFT_DUTY, H_TRACK_LF04_CENTER_RIGHT_DUTY,
    H_TRACK_LF04_SMALL_INNER_DUTY, H_TRACK_LF04_SMALL_OUTER_DUTY,
    H_TRACK_LF04_BIG_INNER_DUTY, H_TRACK_LF04_BIG_OUTER_DUTY,
    H_TRACK_LF04_SHARP_INNER_DUTY, H_TRACK_LF04_SHARP_OUTER_DUTY,
    H_TRACK_LF04_APPROACH_DUTY, H_TRACK_LF04_FINISH_DUTY,
    H_TRACK_LF04_MARKER_CONFIRM_MS, 0U, 0U, 0U
};
volatile uint32_t g_joint_distance_cm_x100;

static uint32_t task2_abs_count(int32_t count)
{
    return (count < 0) ? (uint32_t) (-count) : (uint32_t) count;
}

static uint32_t task2_wheel_distance_cm_x100(uint32_t count,
                                             uint32_t countsPer100Cm)
{
    return ((count * 10000U) + countsPer100Cm / 2U) / countsPer100Cm;
}

static uint32_t task2_forward_distance_cm_x100(int32_t left, int32_t right)
{
    uint32_t leftCm = task2_wheel_distance_cm_x100(task2_abs_count(left),
        H_TRACK_LEFT_ENCODER_COUNTS_PER_100_CM);
    uint32_t rightCm = task2_wheel_distance_cm_x100(task2_abs_count(right),
        H_TRACK_RIGHT_ENCODER_COUNTS_PER_100_CM);
    return leftCm / 2U + rightCm / 2U + ((leftCm & rightCm) & 1U);
}

static void task2_command_to_duty(const HTrackCommand *command,
                                  uint8_t *leftDuty, uint8_t *rightDuty)
{
    if ((command->drive == H_TRACK_DRIVE_LINE_TURN) ||
        (command->drive == H_TRACK_DRIVE_HOLD_LAST)) {
        if (command->direction < 0) {
            *leftDuty = command->innerDuty; *rightDuty = command->outerDuty;
        } else {
            *leftDuty = command->outerDuty; *rightDuty = command->innerDuty;
        }
    } else if (command->drive == H_TRACK_DRIVE_CENTER) {
        *leftDuty = command->innerDuty; *rightDuty = command->outerDuty;
    } else {
        *leftDuty = 0U; *rightDuty = 0U;
    }
}
/* PWM rises/falls by at most 1% per 20-ms speed update. This avoids a
 * static-friction jump at the fourth-S1 start while leaving gains unchanged. */
/* First-drive mode: line PD maps through the S-curve speed reference directly
 * to PWM.  There is intentionally no wheel speed PID correction here. */
static const JointWheelOpenLoopConfig k_wheel_open_loop = {
    CRUISE_SPEED_X100, 22U, 1U
};
static const JointMotionConfig k_motion = {CRUISE_SPEED_X100,
                                            ACCEL_LIMIT_X100,
                                            JERK_LIMIT_X100};
static const JointBeamFeedforwardConfig k_beam = {
    KFF_MILLI, KFF_SIGN, BEAM_CAL_MIN_ANGLE_MDEG, BEAM_CAL_MAX_ANGLE_MDEG,
    20, 150, 250
};
static void wait_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) delay_cycles(MCLK_CYCLES_PER_MS);
}

static void buttons_init(void)
{
    DL_GPIO_enablePower(START_PORT); delay_cycles(16U);
    DL_GPIO_initDigitalInputFeatures(START_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(DOWN_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(UP_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

static void vision_status_led_init(void)
{
    DL_GPIO_enablePower(VISION_RED_LED_PORT);
    DL_GPIO_enablePower(VISION_GREEN_LED_PORT);
    delay_cycles(16U);
    DL_GPIO_initDigitalOutput(VISION_RED_LED_IOMUX);
    DL_GPIO_initDigitalOutput(VISION_GREEN_LED_IOMUX);
    /* LED1 on PA0 is active-low; the green RGB channel is active-high. */
    DL_GPIO_setPins(VISION_RED_LED_PORT, VISION_RED_LED_PIN);
    DL_GPIO_clearPins(VISION_GREEN_LED_PORT, VISION_GREEN_LED_PIN);
    DL_GPIO_enableOutput(VISION_RED_LED_PORT, VISION_RED_LED_PIN);
    DL_GPIO_enableOutput(VISION_GREEN_LED_PORT, VISION_GREEN_LED_PIN);
}

static void show_vision_status(bool visionFresh)
{
    if (visionFresh) {
        DL_GPIO_setPins(VISION_RED_LED_PORT, VISION_RED_LED_PIN);
        DL_GPIO_setPins(VISION_GREEN_LED_PORT, VISION_GREEN_LED_PIN);
    } else {
        DL_GPIO_clearPins(VISION_GREEN_LED_PORT, VISION_GREEN_LED_PIN);
        DL_GPIO_clearPins(VISION_RED_LED_PORT, VISION_RED_LED_PIN);
    }
}

static bool pressed(uint32_t pin)
{
    return DL_GPIO_readPins(START_PORT, pin) == 0U;
}

static const char *state_name(JointState state)
{
    switch (state) {
    case JOINT_WAIT_LEVEL: return "WAIT_LEVEL"; case JOINT_READY: return "READY";
    case JOINT_RAMP_UP: return "RAMP_UP"; case JOINT_CRUISE: return "CRUISE";
    default: return "FAULT";
    }
}

static int32_t pixel_to_cm_x100(int16_t x)
{
    return (x >= 0) ? ((int32_t) x * 1000 / 118) : ((int32_t) x * 100 / 13);
}

static int32_t pixel_rate_to_cm_per_s_x100(int32_t pixelsPerSecond)
{
    return (pixelsPerSecond >= 0) ? (pixelsPerSecond * 1000 / 118) :
                                    (pixelsPerSecond * 100 / 13);
}

static void oled_text(char line[OLED_TEXT_LINE_MAX_CHARS + 1U],
                      const char *text)
{
    uint8_t index = 0U;
    while ((text[index] != '\0') && (index < OLED_TEXT_LINE_MAX_CHARS)) {
        line[index] = text[index]; ++index;
    }
    line[index] = '\0';
}

static void oled_append_text(char line[OLED_TEXT_LINE_MAX_CHARS + 1U],
                             const char *text)
{
    uint8_t length = 0U, index = 0U;
    while ((length < OLED_TEXT_LINE_MAX_CHARS) && line[length] != '\0') ++length;
    while ((text[index] != '\0') && (length < OLED_TEXT_LINE_MAX_CHARS))
        line[length++] = text[index++];
    line[length] = '\0';
}

static void oled_append_int(char line[OLED_TEXT_LINE_MAX_CHARS + 1U],
                            int32_t value)
{
    char reverse[11]; uint8_t length = 0U, count = 0U; uint32_t magnitude;
    while ((length < OLED_TEXT_LINE_MAX_CHARS) && line[length] != '\0') ++length;
    if (value < 0) {
        if (length < OLED_TEXT_LINE_MAX_CHARS) line[length++] = '-';
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        if (length < OLED_TEXT_LINE_MAX_CHARS) line[length++] = '+';
        magnitude = (uint32_t) value;
    }
    do { reverse[count++] = (char) ('0' + magnitude % 10U); magnitude /= 10U; }
    while ((magnitude != 0U) && count < sizeof(reverse));
    while ((count != 0U) && (length < OLED_TEXT_LINE_MAX_CHARS))
        line[length++] = reverse[--count];
    line[length] = '\0';
}

static void oled_append_unsigned(char line[OLED_TEXT_LINE_MAX_CHARS + 1U],
                                 uint32_t value)
{
    char reverse[10]; uint8_t length = 0U, count = 0U;
    while ((length < OLED_TEXT_LINE_MAX_CHARS) && line[length] != '\0') ++length;
    do { reverse[count++] = (char) ('0' + value % 10U); value /= 10U; }
    while ((value != 0U) && count < sizeof(reverse));
    while ((count != 0U) && (length < OLED_TEXT_LINE_MAX_CHARS))
        line[length++] = reverse[--count];
    line[length] = '\0';
}

static void update_oled(JointState state, uint32_t visionFrame,
                        int32_t visionAgeMs, int32_t ballErrorCmX100,
                        int16_t lineError, int16_t lineCorrection,
                        int32_t leftTarget, int32_t leftSpeed,
                        int32_t rightTarget, int32_t rightSpeed,
                        int16_t beamAngleMdeg, uint32_t faultFlags,
                        bool visionFresh, JointLevelStage levelStage,
                        uint16_t elapsedMs)
{
    char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U] = {{0}};
    oled_text(lines[0], "T456 "); oled_append_text(lines[0], state_name(state));
    if (state == JOINT_WAIT_LEVEL) {
        if (levelStage == LEVEL_LOCKED) {
            oled_text(lines[1], "PURE LINE FOLLOW");
            oled_text(lines[2], "NO AUTO STOP");
            oled_text(lines[3], "S1: ARM");
        } else if (levelStage == LEVEL_CONFIRM_LOWER_STOP) {
            oled_text(lines[1], "S1 LIFTS DIRECTLY");
            oled_text(lines[2], "NO POS=0 CHECK");
            oled_text(lines[3], "RELATIVE -565 STEPS");
        } else if (levelStage == LEVEL_PRESET_LIFT) {
            oled_text(lines[1], "LIFTING TO -565");
            oled_text(lines[2], "WAIT - DO NOT TOUCH");
        } else {
            oled_text(lines[1], "FINE LEVEL BEAM");
            oled_text(lines[2], "S2 HOLD: DOWN");
            oled_text(lines[3], "S3 HOLD: UP");
            oled_text(lines[4], "K230 OPTIONAL");
            oled_text(lines[5], "NOWx100 "); oled_append_int(lines[5], ballErrorCmX100);
            oled_text(lines[6], "S1: SAVE LEVEL");
        }
        if (levelStage != LEVEL_FINE_ADJUST)
            oled_text(lines[6], "WHEELS REMAIN LOCKED");
    } else if (state == JOINT_READY) {
        oled_text(lines[1], visionFresh ? "K230 OK - GREEN LED" :
                                            "NO K230 - RED LED");
        oled_text(lines[2], "NEXT: S1 START");
        oled_text(lines[4], "ERRx100 "); oled_append_int(lines[4], ballErrorCmX100);
        oled_text(lines[5], "VIS "); oled_append_unsigned(lines[5], visionFrame);
        oled_text(lines[6], "AGE "); oled_append_int(lines[6], visionAgeMs);
    } else if ((state == JOINT_RAMP_UP) || (state == JOINT_CRUISE)) {
        oled_text(lines[1], "RUNNING - WATCH BALL");
        oled_text(lines[2], "S1: EMERGENCY STOP");
        oled_text(lines[3], "BALLx100 "); oled_append_int(lines[3], ballErrorCmX100);
        oled_text(lines[4], "LINE "); oled_append_int(lines[4], lineError);
        oled_append_text(lines[4], " U"); oled_append_int(lines[4], lineCorrection);
        oled_text(lines[5], "L "); oled_append_int(lines[5], leftTarget);
        oled_append_text(lines[5], "/"); oled_append_int(lines[5], leftSpeed);
        oled_text(lines[6], "R "); oled_append_int(lines[6], rightTarget);
        oled_append_text(lines[6], "/"); oled_append_int(lines[6], rightSpeed);
        oled_text(lines[7], "BEAM "); oled_append_int(lines[7], beamAngleMdeg);
    } else {
        oled_text(lines[1], "FAULT - MOTORS STOP");
        oled_text(lines[2], "NEXT: CHECK LOG");
        oled_text(lines[3], "FIX CAUSE THEN RESET");
        oled_text(lines[5], "FAULT "); oled_append_unsigned(lines[5], faultFlags);
        if ((faultFlags & FAULT_VISION_LOST) != 0U) oled_text(lines[6], "K230/UART LOST");
        else if ((faultFlags & FAULT_BALL_LIMIT) != 0U) oled_text(lines[6], "BALL OVER +/-1CM");
        else if ((faultFlags & FAULT_LINE_LOST) != 0U) oled_text(lines[6], "BLACK LINE LOST");
        else if ((faultFlags & FAULT_ENCODER_STALL) != 0U) oled_text(lines[6], "ENCODER/WHEEL ERROR");
        else if ((faultFlags & FAULT_STEPPER) != 0U) oled_text(lines[6], "STEPPER ERROR");
        else if ((faultFlags & FAULT_EMERGENCY) != 0U) oled_text(lines[6], "S1 EMERGENCY STOP");
    }
    Oled_updateTextLines(lines, elapsedMs);
}

#if defined(COMPETITION_UNIFIED_ENTRY)
void Joint456_run(void)
#else
int main(void)
#endif
{
    JointState state = JOINT_WAIT_LEVEL;
    JointLevelStage levelStage = LEVEL_LOCKED;
    JointLinePd line; JointWheelOpenLoop leftWheel, rightWheel; JointMotionProfile motion;
    HTrackController task2Line;
    JointAccelerationEstimator accel; JointBeamFeedforward beam;
    BallBeamController ballController; BallBeamCommand ballCommand = {0};
    VisionBallMeasurement vision = {0}; Icm42688Sample imu = {0};
    StepperBeamStatus stepper; BeamCalibrationInterpolation interpolation;
    uint32_t now = 0U, lastVisionMs = 0U, previousVisionMs = 0U;
    uint32_t centeredSinceMs = 0U, runId = 0U, runStartedMs = 0U;
    uint32_t faultFlags = 0U, releaseMs = 0U;
    int32_t lastLeft = 0, lastRight = 0, leftSpeed = 0, rightSpeed = 0;
    int32_t leftTarget = 0, rightTarget = 0, vehicleSpeed = 0, vehicleAccel = 0;
    int32_t ballPosition = 0, ballReference = 0;
    int32_t ballError = 0, ballVelocity = 0, previousBallPosition = 0;
    int16_t lineError = 0, beamAngle = 0, ballPdAngle = 0; uint8_t rawMask = 0U;
    uint8_t task2RequestedLeft = 0U, task2RequestedRight = 0U;
    uint8_t task2LeftDuty = 0U, task2RightDuty = 0U;
    bool haveVision = false, imuValid = false, ballReferenceSaved = false;
    bool startLatched = false;
    int32_t levelPosition = 0;

    Motor_init(); Motor_stopAll(); Encoder_init(); Encoder_resetCounts();
    buttons_init(); vision_status_led_init();
    IrLineSensor_init(); VisionUart_init(); DiagnosticUart_init();
    StepperBeam_init(); StepperBeam_setArmed(false); (void) Icm42688_init();
    wait_ms(200U); (void) Oled_init();
    JointLine_init(&line, &k_line);
    HTrackController_init(&task2Line, &k_task2_line);
    HTrackController_setAMarkerArmed(&task2Line, 0U);
    JointWheelOpenLoop_init(&leftWheel, &k_wheel_open_loop);
    JointWheelOpenLoop_init(&rightWheel, &k_wheel_open_loop);
    JointMotion_init(&motion, &k_motion);
    JointAcceleration_init(&accel); JointBeamFeedforward_init(&beam, &k_beam);
    BallBeamController_init(&ballController, BALL_BEAM_CAMERA_CENTER_X_PX);

    for (;;) {
        bool start = pressed(START_PIN);
        bool startEdge = false;
        if (start) {
            /* Respond on the first falling edge. Re-arm only after a stable
             * release, so contact bounce cannot create another S1 action. */
            if (!startLatched) {
                startEdge = true;
                startLatched = true;
            }
            releaseMs = 0U;
        } else if (startLatched) {
            if (releaseMs < BUTTON_DEBOUNCE_MS) releaseMs += TICK_MS;
            if (releaseMs >= BUTTON_DEBOUNCE_MS) {
                startLatched = false;
                releaseMs = 0U;
            }
        }
        StepperBeam_service(); StepperBeam_getStatus(&stepper);
        if ((levelStage == LEVEL_PRESET_LIFT) && (stepper.moving == 0U))
            levelStage = LEVEL_FINE_ADJUST;
        if (VisionUart_pollNewest(&vision)) {
            int32_t newPosition = pixel_to_cm_x100(vision.xOffsetPx);
            uint32_t visionDt = now - previousVisionMs;
            ballVelocity = (previousVisionMs != 0U && visionDt != 0U) ?
                (newPosition - previousBallPosition) * 1000 / (int32_t) visionDt : 0;
            previousBallPosition = newPosition; ballPosition = newPosition;
            ballError = ballReferenceSaved ? ballPosition - ballReference : ballPosition;
            previousVisionMs = now; lastVisionMs = now; haveVision = (vision.lost == 0U) &&
                (vision.confidenceMilli >= VISION_MIN_CONFIDENCE);
            if (ballReferenceSaved && haveVision &&
                BallBeamController_update(&ballController, vision.xOffsetPx,
                    now, BALL_CONTROL_SIGN, &ballCommand)) {
                /* Keep task 3's shared controller frozen at Kp=11 and apply
                 * only the joint image's additional proportional action. */
                int32_t jointAngleMdeg =
                    (int32_t) ballCommand.requestedAngleMdeg +
                    (int32_t) (JOINT_BALL_KP_MDEG_PER_PX -
                               BALL_BEAM_KP_MDEG_PER_PX) *
                    ballCommand.errorPx * BALL_CONTROL_SIGN;
                if (jointAngleMdeg > BALL_PD_LIMIT_MDEG)
                    ballCommand.requestedAngleMdeg = BALL_PD_LIMIT_MDEG;
                else if (jointAngleMdeg < -BALL_PD_LIMIT_MDEG)
                    ballCommand.requestedAngleMdeg = -BALL_PD_LIMIT_MDEG;
                else ballCommand.requestedAngleMdeg = (int16_t) jointAngleMdeg;
                ballVelocity = pixel_rate_to_cm_per_s_x100(
                    ballCommand.velocityPxPerS);
            }
            if (haveVision && ballError >= -BALL_LIMIT_CM_X100 &&
                ballError <= BALL_LIMIT_CM_X100) {
                if (centeredSinceMs == 0U) centeredSinceMs = now;
            } else centeredSinceMs = 0U;
            /* If the third S1 was accepted without K230, acquire the first
             * later valid measurement as this run's hold target. */
            if ((levelStage == LEVEL_SAVED) && !ballReferenceSaved && haveVision) {
                ballReference = ballPosition;
                ballError = 0;
                ballReferenceSaved = true;
                BallBeamController_init(&ballController,
                    (int16_t) (BALL_BEAM_CAMERA_CENTER_X_PX + vision.xOffsetPx));
                ballCommand.requestedAngleMdeg = 0;
                centeredSinceMs = now;
            }
        }

        show_vision_status(haveVision && (now - lastVisionMs <= VISION_TIMEOUT_MS));

        if (state == JOINT_WAIT_LEVEL) {
            Motor_stopAll();
            if ((levelStage == LEVEL_FINE_ADJUST) &&
                (pressed(UP_PIN) != pressed(DOWN_PIN))) {
                (void) StepperBeam_startManualJog(pressed(UP_PIN) ? 1 : -1,
                    LEVEL_MANUAL_JOG_PPS,
                    LOWER_STOP_TO_LEVEL_STEPS + BEAM_CAL_MIN_POS,
                    LOWER_STOP_TO_LEVEL_STEPS + BEAM_CAL_MAX_POS);
            } else if ((levelStage == LEVEL_FINE_ADJUST) &&
                       (stepper.moving != 0U)) {
                StepperBeam_stop(); StepperBeam_setArmed(true);
            }
            if (startEdge && (levelStage == LEVEL_LOCKED)) {
                StepperBeam_setArmed(true);
                levelStage = LEVEL_CONFIRM_LOWER_STOP;
            } else if (startEdge && (levelStage == LEVEL_CONFIRM_LOWER_STOP) &&
                       StepperBeam_queueJog(LOWER_STOP_TO_LEVEL_STEPS)) {
                levelStage = LEVEL_PRESET_LIFT;
            } else if (startEdge && (levelStage == LEVEL_FINE_ADJUST) &&
                       (stepper.moving == 0U) &&
                       StepperBeam_acceptManualReference()) {
                StepperBeam_getStatus(&stepper); levelPosition = stepper.positionSteps;
                if (StepperBeam_configureTravelLimits(levelPosition + BEAM_CAL_MIN_POS,
                                                       levelPosition + BEAM_CAL_MAX_POS)) {
                    /* K230 is optional for starting. When fresh, the third S1
                     * also saves the ball target; otherwise it is acquired
                     * automatically from the first later valid frame. */
                    if (haveVision && (now - lastVisionMs <= VISION_TIMEOUT_MS)) {
                        ballReference = ballPosition;
                        ballError = 0;
                        ballReferenceSaved = true;
                        BallBeamController_init(&ballController,
                            (int16_t) (BALL_BEAM_CAMERA_CENTER_X_PX + vision.xOffsetPx));
                    } else {
                        ballReferenceSaved = false;
                    }
                    ballCommand.requestedAngleMdeg = 0;
                    centeredSinceMs = ballReferenceSaved ? now : 0U;
                    levelStage = LEVEL_SAVED;
                    state = JOINT_READY;
                }
            }
        } else if (state == JOINT_READY) {
            /* The 20-ms beam update owns the target here, so a ball moved
             * after third S1 is returned to that saved reference. */
            Motor_stopAll();
            if (startEdge) {
                ++runId; Encoder_resetCounts(); lastLeft = lastRight = 0;
                runStartedMs = now;
                g_joint_distance_cm_x100 = 0U;
                HTrackController_init(&task2Line, &k_task2_line);
                HTrackController_setAMarkerArmed(&task2Line, 0U);
                task2RequestedLeft = task2RequestedRight = 0U;
                task2LeftDuty = task2RightDuty = 0U;
                JointMotion_init(&motion, &k_motion);
                state = JOINT_RAMP_UP;
            }
        } else if ((state == JOINT_RAMP_UP) || (state == JOINT_CRUISE)) {
            if (startEdge) { faultFlags |= FAULT_EMERGENCY; state = JOINT_FAULT;
                Motor_stopAll(); StepperBeam_stop(); StepperBeam_setArmed(false); }
            /* Line-first test mode: ball error remains fully logged and the
             * vision PD remains active, but ball position cannot stop wheels. */
            if (faultFlags != 0U && (faultFlags & FAULT_EMERGENCY) == 0U) {
                Motor_stopAll(); JointMotion_setRunning(&motion, false);
                state = JOINT_FAULT;
            }
        }

        rawMask = IrLineSensor_readStableRawMask();
        {
            uint8_t dhState = IrLineSensor_rawToDhWhiteState(rawMask, 0U);
            if ((state == JOINT_RAMP_UP) || (state == JOINT_CRUISE)) {
                HTrackCommand command = HTrackController_step(
                    &task2Line, dhState, 0U, TICK_MS);
                task2_command_to_duty(&command, &task2RequestedLeft,
                                      &task2RequestedRight);
            }
            lineError = (int16_t) task2RightDuty - (int16_t) task2LeftDuty;
            line.correction = lineError;
        }

        if ((now % SPEED_PERIOD_MS) == 0U) {
            int32_t left = g_motorA_encoder_count, right = g_motorB_encoder_count;
            int32_t leftDelta = left - lastLeft, rightDelta = right - lastRight;
            uint32_t startupElapsedMs = now - runStartedMs;
            uint32_t startupPrepareMs = STARTUP_PWM_RAMP_MS +
                                        STARTUP_PRETILT_HOLD_MS;
            g_joint_distance_cm_x100 =
                task2_forward_distance_cm_x100(left, right);
            /* Current forward encoder polarity is negative in verified calibration. */
            leftSpeed = (-leftDelta * 10000000L) / (LEFT_COUNTS_PER_100_CM * SPEED_PERIOD_MS);
            rightSpeed = (-rightDelta * 10000000L) / (RIGHT_COUNTS_PER_100_CM * SPEED_PERIOD_MS);
            lastLeft = left; lastRight = right;
            if (((state == JOINT_RAMP_UP) || (state == JOINT_CRUISE)) &&
                (startupElapsedMs >= STARTUP_FEEDFORWARD_BEGIN_MS) &&
                (motion.targetSpeedX100 == 0))
                JointMotion_setRunning(&motion, true);
            JointMotion_step(&motion, SPEED_PERIOD_MS);
            if ((state == JOINT_RAMP_UP) &&
                (startupElapsedMs < STARTUP_PWM_RAMP_MS)) {
                task2LeftDuty = (uint8_t) (((uint32_t) STARTUP_LEFT_BELOW_DUTY *
                    startupElapsedMs) / STARTUP_PWM_RAMP_MS);
                task2RightDuty = (uint8_t) (((uint32_t) STARTUP_RIGHT_BELOW_DUTY *
                    startupElapsedMs) / STARTUP_PWM_RAMP_MS);
            } else if ((state == JOINT_RAMP_UP) &&
                       (startupElapsedMs < startupPrepareMs)) {
                task2LeftDuty = STARTUP_LEFT_BELOW_DUTY;
                task2RightDuty = STARTUP_RIGHT_BELOW_DUTY;
            } else {
                task2LeftDuty = (uint8_t) (((uint32_t) task2RequestedLeft *
                    (uint32_t) motion.speedX100) / CRUISE_SPEED_X100);
                task2RightDuty = (uint8_t) (((uint32_t) task2RequestedRight *
                    (uint32_t) motion.speedX100) / CRUISE_SPEED_X100);
                /* Cross the measured static-friction boundary by only 1 PWM
                 * point. A deliberately stopped inner wheel remains zero. */
                if ((task2RequestedLeft != 0U) &&
                    (task2LeftDuty < STARTUP_LEFT_RUN_DUTY))
                    task2LeftDuty = STARTUP_LEFT_RUN_DUTY;
                if ((task2RequestedRight != 0U) &&
                    (task2RightDuty < STARTUP_RIGHT_RUN_DUTY))
                    task2RightDuty = STARTUP_RIGHT_RUN_DUTY;
            }
            if ((state == JOINT_RAMP_UP) &&
                (motion.speedX100 == CRUISE_SPEED_X100)) state = JOINT_CRUISE;
            if ((state == JOINT_RAMP_UP) || (state == JOINT_CRUISE)) {
                Motor_setBothDuty(MOTOR_DIRECTION_FORWARD,
                    MOTOR_DIRECTION_FORWARD, task2LeftDuty, task2RightDuty);
            }
            leftTarget = task2LeftDuty;
            rightTarget = task2RightDuty;
            vehicleSpeed = (leftSpeed + rightSpeed) / 2;
            vehicleAccel = JointAcceleration_step(&accel, vehicleSpeed, SPEED_PERIOD_MS);
            beamAngle = JointBeamFeedforward_step(&beam,
                motion.accelerationX100, line.correction, 0);
            /* Pre-position before either wheel reaches breakaway. Keep the
             * same conservative angle through ramp-up until planned FF takes
             * over; visual PD remains independently active. */
            if ((state == JOINT_RAMP_UP) &&
                (beamAngle * KFF_SIGN < STARTUP_PRETILT_MDEG))
                beamAngle = (int16_t) (STARTUP_PRETILT_MDEG * KFF_SIGN);
            ballPdAngle = (ballReferenceSaved && haveVision &&
                (now - lastVisionMs <= VISION_TIMEOUT_MS)) ?
                ballCommand.requestedAngleMdeg : 0;
            beamAngle = (int16_t) ((int32_t) beamAngle + ballPdAngle);
            if (beamAngle < BEAM_CAL_MIN_ANGLE_MDEG) beamAngle = BEAM_CAL_MIN_ANGLE_MDEG;
            if (beamAngle > BEAM_CAL_MAX_ANGLE_MDEG) beamAngle = BEAM_CAL_MAX_ANGLE_MDEG;
            if (BeamCalibration_angleToPos(beamAngle, &interpolation))
                (void) StepperBeam_setTargetSteps(levelPosition + interpolation.targetPos);
            /* Encoders remain telemetry/feed-forward inputs in line-first
             * mode; they do not veto the open-loop PWM command. */
        }

        if ((now % IMU_PERIOD_MS) == 0U) imuValid = Icm42688_readSample(&imu);
        if (stepper.fault != STEPPER_BEAM_FAULT_NONE) {
            faultFlags |= FAULT_STEPPER; Motor_stopAll(); StepperBeam_stop(); state = JOINT_FAULT;
        }
        if ((now % LOG_PERIOD_MS) == 0U) {
            DiagnosticJointTelemetry t = {0}; StepperBeam_getStatus(&stepper);
            t.mcuMs=now; t.runId=runId; t.state=state_name(state);
            t.safetyState=(faultFlags==0U)?"OK":"FAULT_LATCHED"; t.irRawMask=rawMask;
            t.lineError=lineError; t.lineErrorRate=line.filteredRate; t.lineCorrection=line.correction;
            t.baseSpeedRef=motion.speedX100; t.baseAccelerationRef=motion.accelerationX100;
            t.leftSpeedRef=leftTarget; t.rightSpeedRef=rightTarget; t.leftSpeed=leftSpeed; t.rightSpeed=rightSpeed;
            t.leftSpeedError=leftTarget-leftSpeed; t.rightSpeedError=rightTarget-rightSpeed;
            t.leftPwm=task2LeftDuty; t.rightPwm=task2RightDuty;
            t.leftEncoder=g_motorA_encoder_count; t.rightEncoder=g_motorB_encoder_count;
            t.vehicleSpeed=vehicleSpeed; t.vehicleAcceleration=vehicleAccel;
            t.imuAccelX=imuValid?imu.accelX:IMU_MISSING; t.imuAccelY=imuValid?imu.accelY:IMU_MISSING;
            t.imuAccelZ=imuValid?imu.accelZ:IMU_MISSING; t.kffMilli=KFF_MILLI;
            t.beamFeedforwardMdeg=beam.angleMdeg; t.beamBallPdMdeg=ballPdAngle; t.beamTargetMdeg=beamAngle;
            t.stepperTarget=stepper.targetSteps; t.stepperPosition=stepper.positionSteps;
            t.stepFrequency=stepper.pulseRatePps; t.visionFrame=vision.frame;
            t.visionAgeMs=haveVision?(int32_t)(now-lastVisionMs):-1;
            t.ballTargetCmX100=ballReference;
            t.ballErrorCmX100=ballError; t.ballVelocityCmPerSX100=ballVelocity;
            t.faultFlags=faultFlags; DiagnosticUart_writeJointCsv(&t);
        }
        update_oled(state, vision.frame,
            haveVision ? (int32_t) (now - lastVisionMs) : -1,
            ballError, lineError, line.correction,
            leftTarget, leftSpeed, rightTarget, rightSpeed,
            beamAngle, faultFlags,
            haveVision && (now - lastVisionMs <= VISION_TIMEOUT_MS) &&
                centeredSinceMs != 0U &&
                (now - centeredSinceMs >= BALL_CENTER_DWELL_MS),
            levelStage, TICK_MS);
        wait_ms(TICK_MS); now += TICK_MS;
    }
}
