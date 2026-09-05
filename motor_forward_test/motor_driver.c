#include "motor_driver.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "encoder.h"

/* TB6612 inputs, all on the numbered 40-pin LaunchPad header. */
#define MOTOR_A_IN1_PIN       DL_GPIO_PIN_8   /* AIN1 -> PA8 */
#define MOTOR_A_IN2_PIN       DL_GPIO_PIN_17  /* AIN2 -> PB17 */
#define MOTOR_B_IN1_PIN       DL_GPIO_PIN_23  /* BIN1 -> PB23 / SW1 */
#define MOTOR_B_IN2_PIN       DL_GPIO_PIN_25  /* BIN2 -> PA25 */
#define MOTOR_PWM_GPIO_PINS   (DL_GPIO_PIN_1 | DL_GPIO_PIN_4)

/* PB4 = TIMA1 CCP0 -> PWMA; PB1 = TIMA1 CCP1 -> PWMB. */
#define MOTOR_PWM_TIMER               TIMA1
#define MOTOR_PWM_PERIOD_TICKS        200U  /* 4 MHz / 200 = 20 kHz */
#define MOTOR_PWM_STOP_COMPARE         (MOTOR_PWM_PERIOD_TICKS - 1U)

/* 32-MHz MCLK keeps the existing 1-ms control-slice timing. */
#define CONTROL_SLICE_CYCLES         32000U
#define ENCODER_SAMPLE_CYCLES          256U
#define DIRECTION_CHANGE_DEADTIME       64U  /* 2 us with bridge released */

/*
 * Physical vehicle convention: "FORWARD" means the chassis moves toward the
 * LF04 sensor bar (the intended course direction).  The rear-wheel assembly
 * is wired such that IN1=high drove the chassis backwards, so both bridge
 * direction polarities are inverted here.  Keep the application code using
 * MOTOR_DIRECTION_FORWARD; do not compensate by making the vehicle follow
 * the course backwards.
 */
#define MOTOR_A_FORWARD_IN1_HIGH   0U
#define MOTOR_B_FORWARD_IN1_HIGH   0U

static const DL_TimerA_ClockConfig g_motor_pwm_clock_config = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U,
};

static const DL_TimerA_PWMConfig g_motor_pwm_config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = MOTOR_PWM_PERIOD_TICKS,
    .isTimerWithFourCC = false,
    /* Counter is explicitly started after both PWM outputs are configured. */
    .startTimer = DL_TIMER_STOP,
};

/* Useful live-debug symbols in CCS: requested continuous PWM duty. */
volatile uint8_t g_motor_pwm_a_duty = 0U;
volatile uint8_t g_motor_pwm_b_duty = 0U;
volatile uint8_t g_motor_pwm_continuous = 0U;

static uint8_t motor_limit_duty(uint8_t dutyPercent)
{
    return (dutyPercent > 100U) ? 100U : dutyPercent;
}

static void motor_clear_direction(MotorChannel channel)
{
    if (channel == MOTOR_CHANNEL_A) {
        DL_GPIO_clearPins(GPIOA, MOTOR_A_IN1_PIN);
        DL_GPIO_clearPins(GPIOB, MOTOR_A_IN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIOB, MOTOR_B_IN1_PIN);
        DL_GPIO_clearPins(GPIOA, MOTOR_B_IN2_PIN);
    }
}

static void motor_set_direction(MotorChannel channel, MotorDirection direction)
{
    const uint32_t directionIsForward = (direction == MOTOR_DIRECTION_FORWARD);

    if (channel == MOTOR_CHANNEL_A) {
        const uint32_t in1High = directionIsForward ?
            MOTOR_A_FORWARD_IN1_HIGH : !MOTOR_A_FORWARD_IN1_HIGH;

        if (in1High != 0U) {
            DL_GPIO_setPins(GPIOA, MOTOR_A_IN1_PIN);
            DL_GPIO_clearPins(GPIOB, MOTOR_A_IN2_PIN);
        } else {
            DL_GPIO_clearPins(GPIOA, MOTOR_A_IN1_PIN);
            DL_GPIO_setPins(GPIOB, MOTOR_A_IN2_PIN);
        }
    } else {
        const uint32_t in1High = directionIsForward ?
            MOTOR_B_FORWARD_IN1_HIGH : !MOTOR_B_FORWARD_IN1_HIGH;

        if (in1High != 0U) {
            DL_GPIO_setPins(GPIOB, MOTOR_B_IN1_PIN);
            DL_GPIO_clearPins(GPIOA, MOTOR_B_IN2_PIN);
        } else {
            DL_GPIO_clearPins(GPIOB, MOTOR_B_IN1_PIN);
            DL_GPIO_setPins(GPIOA, MOTOR_B_IN2_PIN);
        }
    }
}

static void motor_set_pwm_duty(MotorChannel channel, uint8_t dutyPercent)
{
    const uint8_t limitedDuty = motor_limit_duty(dutyPercent);
    uint32_t compareValue = MOTOR_PWM_STOP_COMPARE;

    if (limitedDuty != 0U) {
        const uint32_t highTicks =
            (MOTOR_PWM_PERIOD_TICKS * (uint32_t) limitedDuty) / 100U;

        /* Edge-aligned PWM is high from LOAD to the down-count compare. */
        compareValue = MOTOR_PWM_PERIOD_TICKS - highTicks;
    }

    if (channel == MOTOR_CHANNEL_A) {
        DL_TimerA_setCaptureCompareValue(MOTOR_PWM_TIMER, compareValue,
                                         DL_TIMER_CC_0_INDEX);
        g_motor_pwm_a_duty = limitedDuty;
    } else {
        DL_TimerA_setCaptureCompareValue(MOTOR_PWM_TIMER, compareValue,
                                         DL_TIMER_CC_1_INDEX);
        g_motor_pwm_b_duty = limitedDuty;
    }
}

static void motor_apply(MotorChannel channel, MotorDirection direction,
                        uint8_t dutyPercent)
{
    const uint8_t limitedDuty = motor_limit_duty(dutyPercent);

    if (limitedDuty == 0U) {
        /* PWM stays generated in hardware; bridge inputs release this wheel. */
        motor_set_pwm_duty(channel, 0U);
        motor_clear_direction(channel);
        return;
    }

    /*
     * Release briefly before a possible polarity reversal.  For the normal
     * forward-only line tracker this is just a redundant, harmless GPIO set;
     * for spin tests it protects the TB6612 against shoot-through.
     */
    motor_clear_direction(channel);
    delay_cycles(DIRECTION_CHANGE_DEADTIME);
    motor_set_direction(channel, direction);
    motor_set_pwm_duty(channel, limitedDuty);
}

static void motor_delay_with_encoder_sampling(uint32_t cycles)
{
    while (cycles >= ENCODER_SAMPLE_CYCLES) {
        Encoder_sample();
        delay_cycles(ENCODER_SAMPLE_CYCLES);
        cycles -= ENCODER_SAMPLE_CYCLES;
    }

    if (cycles != 0U) {
        Encoder_sample();
        delay_cycles(cycles);
    }
}

static void motor_wait_ms(uint32_t durationMs)
{
    while (durationMs-- != 0U) {
        motor_delay_with_encoder_sampling(CONTROL_SLICE_CYCLES);
    }
}

static void motor_init_pwm_timer(void)
{
    DL_TimerA_setClockConfig(MOTOR_PWM_TIMER,
                             (DL_TimerA_ClockConfig *) &g_motor_pwm_clock_config);
    DL_TimerA_initPWMMode(MOTOR_PWM_TIMER,
                          (DL_TimerA_PWMConfig *) &g_motor_pwm_config);

    /* CC0 is the base channel; CC0/CC1 independently drive the two bridges. */
    DL_TimerA_setCounterControl(MOTOR_PWM_TIMER,
        DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(MOTOR_PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(MOTOR_PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_TIMER, MOTOR_PWM_STOP_COMPARE,
                                     DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(MOTOR_PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(MOTOR_PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_TIMER, MOTOR_PWM_STOP_COMPARE,
                                     DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(MOTOR_PWM_TIMER);
    DL_TimerA_setCCPDirection(MOTOR_PWM_TIMER,
                              DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerA_enableShadowFeatures(MOTOR_PWM_TIMER);
    DL_TimerA_startCounter(MOTOR_PWM_TIMER);
    g_motor_pwm_continuous = 1U;
}

void Motor_init(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(MOTOR_PWM_TIMER);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(MOTOR_PWM_TIMER);
    delay_cycles(16U);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

    /* PWM pins are peripherals, not manually toggled GPIO outputs. */
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM13,
                                         IOMUX_PINCM13_PF_TIMA1_CCP1); /* PB1 */
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM17,
                                         IOMUX_PINCM17_PF_TIMA1_CCP0); /* PB4 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM19); /* PA8 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM51); /* PB23 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM55); /* PA25 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM43); /* PB17 */

    DL_GPIO_clearPins(GPIOA, MOTOR_A_IN1_PIN | MOTOR_B_IN2_PIN);
    DL_GPIO_clearPins(GPIOB, MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN);
    DL_GPIO_enableOutput(GPIOA, MOTOR_A_IN1_PIN | MOTOR_B_IN2_PIN);
    DL_GPIO_enableOutput(GPIOB, MOTOR_PWM_GPIO_PINS | MOTOR_A_IN2_PIN |
                                 MOTOR_B_IN1_PIN);

    motor_init_pwm_timer();
    Motor_stopAll();
}

void Motor_stop(MotorChannel channel)
{
    motor_set_pwm_duty(channel, 0U);
    motor_clear_direction(channel);
}

void Motor_stopAll(void)
{
    Motor_stop(MOTOR_CHANNEL_A);
    Motor_stop(MOTOR_CHANNEL_B);
}

void Motor_brakeAllFor(uint32_t durationMs)
{
    /* TB6612 short-brake: IN1 = IN2 = high, with PWM continuously high. */
    Motor_stopAll();
    motor_set_pwm_duty(MOTOR_CHANNEL_A, 100U);
    motor_set_pwm_duty(MOTOR_CHANNEL_B, 100U);
    DL_GPIO_setPins(GPIOA, MOTOR_A_IN1_PIN | MOTOR_B_IN2_PIN);
    DL_GPIO_setPins(GPIOB, MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN);
    motor_wait_ms(durationMs);
    Motor_stopAll();
}

void Motor_runFor(MotorChannel channel, MotorDirection direction,
                  uint8_t dutyPercent, uint32_t durationMs)
{
    Motor_stop(channel);
    motor_apply(channel, direction, dutyPercent);
    motor_wait_ms(durationMs);
    Motor_stop(channel);
}

void Motor_runBothFor(MotorDirection directionA, MotorDirection directionB,
                      uint8_t dutyPercent, uint32_t durationMs)
{
    Motor_runBothWithDutyFor(directionA, directionB, dutyPercent, dutyPercent,
                              durationMs);
    Motor_stopAll();
}

void Motor_runBothWithDutyFor(MotorDirection directionA,
                              MotorDirection directionB,
                              uint8_t dutyA, uint8_t dutyB,
                              uint32_t durationMs)
{
    /*
     * This is deliberately non-stopping.  The 20-kHz timer continues driving
     * both PWM pins while the caller spends its 4-ms control period reading
     * grey sensors and the IMU.  Only an explicit Motor_stop* call releases a
     * bridge, so low-duty correction has continuous torque instead of pulses.
     */
    motor_apply(MOTOR_CHANNEL_A, directionA, dutyA);
    motor_apply(MOTOR_CHANNEL_B, directionB, dutyB);
    motor_wait_ms(durationMs);
}

void Motor_setBothDuty(MotorDirection directionA, MotorDirection directionB,
                       uint8_t dutyA, uint8_t dutyB)
{
    motor_apply(MOTOR_CHANNEL_A, directionA, dutyA);
    motor_apply(MOTOR_CHANNEL_B, directionB, dutyB);
}
