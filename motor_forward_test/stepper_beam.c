#include "stepper_beam.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* X42S pulse pins. The numbers are the MSPM0G3507 PINCM assignments. */
#define STEPPER_PORT                    GPIOB
#define STEPPER_STP_PIN                 DL_GPIO_PIN_13
#define STEPPER_DIR_PIN                 DL_GPIO_PIN_12
#define STEPPER_EN_PIN                  DL_GPIO_PIN_6
#define STEPPER_STP_IOMUX               IOMUX_PINCM30
#define STEPPER_DIR_IOMUX               IOMUX_PINCM29
#define STEPPER_EN_IOMUX                IOMUX_PINCM23
#define STEPPER_ALL_PINS                (STEPPER_STP_PIN | STEPPER_DIR_PIN | \
                                         STEPPER_EN_PIN)

/* TIMG0 is unused by this isolated image. With a 32-MHz MCLK / 8 it runs at
 * 4 MHz, giving a 20-us active-low pulse = 80 timer ticks. A 16-bit timer can
 * still represent the 10-ms period used by the 100-pulse/s safe start speed. */
#define STEPPER_TIMER                   TIMG0
#define STEPPER_TIMER_IRQ               TIMG0_INT_IRQn
#define STEPPER_TIMER_TICKS_PER_US      4U
#define STEPPER_PULSE_LOW_US            20U
#define STEPPER_PULSE_LOW_TICKS         (STEPPER_PULSE_LOW_US * \
                                         STEPPER_TIMER_TICKS_PER_US)
#define STEPPER_START_PPS               100U
#define STEPPER_MAX_PPS                 800U
#define STEPPER_ACCEL_PPS2              800U

typedef enum {
    MOTION_NONE = 0,
    MOTION_JOG,
    MOTION_MANUAL_JOG,
    MOTION_TRACK,
} MotionKind;

static volatile int32_t s_position_steps;
static volatile int32_t s_target_steps;
static volatile int32_t s_travel_min_steps;
static volatile int32_t s_travel_max_steps;
static volatile int16_t s_remaining_steps;
static volatile uint16_t s_pulse_rate_pps;
static volatile uint8_t s_armed;
static volatile uint8_t s_moving;
static volatile uint8_t s_step_low;
static volatile uint8_t s_move_done;
static volatile uint8_t s_travel_limits_configured;
static volatile int8_t s_direction;
static volatile MotionKind s_motion_kind;
static volatile StepperBeamHomeState s_home_state;
static volatile StepperBeamFault s_fault;
static volatile uint8_t s_timer_ready;
static volatile int32_t s_manual_jog_min_steps;
static volatile int32_t s_manual_jog_max_steps;

static uint32_t irq_lock(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void irq_restore(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void outputs_idle_high(void)
{
    DL_GPIO_setPins(STEPPER_PORT, STEPPER_ALL_PINS);
}

/* Keep TIMG0 completely untouched during cold boot.  It is only needed after
 * the user deliberately arms the X42S.  This leaves OLED and button startup
 * independent of the pulse engine and of any timer reset state. */
static void prepare_pulse_timer(void)
{
    const DL_TimerG_TimerConfig timerConfig = {
        .period = 0xFFFFU,
        .timerMode = DL_TIMER_TIMER_MODE_ONE_SHOT,
        .startTimer = false,
        .counterVal = 0U,
        .genIntermInt = DL_TIMER_INTERM_INT_DISABLED,
    };
    const DL_TimerG_ClockConfig clockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .prescale = 0U,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    };

    if (s_timer_ready != 0U) {
        return;
    }
    DL_TimerG_reset(STEPPER_TIMER);
    DL_TimerG_enablePower(STEPPER_TIMER);
    DL_TimerG_initTimerMode(STEPPER_TIMER,
                            (DL_TimerG_TimerConfig *) &timerConfig);
    DL_TimerG_setClockConfig(STEPPER_TIMER,
                             (DL_TimerG_ClockConfig *) &clockConfig);
    DL_TimerG_clearInterruptStatus(STEPPER_TIMER,
                                   DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableInterrupt(STEPPER_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(STEPPER_TIMER_IRQ);
    DL_TimerG_enableClock(STEPPER_TIMER);
    s_timer_ready = 1U;
}

static uint16_t pps_to_period_ticks(uint16_t pps)
{
    uint32_t ticks;

    if (pps == 0U) {
        pps = STEPPER_START_PPS;
    }
    ticks = 4000000UL / pps;
    if (ticks <= STEPPER_PULSE_LOW_TICKS) {
        ticks = STEPPER_PULSE_LOW_TICKS + 1U;
    }
    if (ticks > 0xFFFFU) {
        ticks = 0xFFFFU;
    }
    return (uint16_t) ticks;
}

static void timer_schedule_ticks(uint16_t ticks)
{
    if (ticks == 0U) {
        ticks = 1U;
    }
    DL_TimerG_stopCounter(STEPPER_TIMER);
    DL_TimerG_setLoadValue(STEPPER_TIMER, (uint16_t) (ticks - 1U));
    DL_TimerG_startCounter(STEPPER_TIMER);
}

static uint16_t next_pulse_rate(uint16_t current, uint16_t stepsLeft)
{
    uint16_t delta;
    uint32_t brakeSteps;

    if (current < STEPPER_START_PPS) {
        current = STEPPER_START_PPS;
    }
    delta = (uint16_t) (STEPPER_ACCEL_PPS2 / current);
    if (delta == 0U) {
        delta = 1U;
    }
    brakeSteps = ((uint32_t) current * (uint32_t) current) /
                 (2U * STEPPER_ACCEL_PPS2) + 1U;
    if ((stepsLeft <= brakeSteps) && (current > STEPPER_START_PPS)) {
        return (current > delta + STEPPER_START_PPS) ?
            (uint16_t) (current - delta) : STEPPER_START_PPS;
    }
    if (current < STEPPER_MAX_PPS) {
        const uint32_t raised = (uint32_t) current + delta;
        return (raised > STEPPER_MAX_PPS) ? STEPPER_MAX_PPS :
            (uint16_t) raised;
    }
    return current;
}

static void begin_motion_locked(int16_t relativeSteps, MotionKind kind,
                                uint16_t initialPps)
{
    const int8_t direction = (relativeSteps >= 0) ? 1 : -1;

    s_direction = direction;
    s_remaining_steps = (relativeSteps >= 0) ? relativeSteps :
        (int16_t) -relativeSteps;
    s_target_steps = s_position_steps + relativeSteps;
    s_motion_kind = kind;
    s_pulse_rate_pps = initialPps;
    s_move_done = 0U;
    s_moving = 1U;
    s_step_low = 0U;
    if (direction > 0) {
        DL_GPIO_setPins(STEPPER_PORT, STEPPER_DIR_PIN);
    } else {
        DL_GPIO_clearPins(STEPPER_PORT, STEPPER_DIR_PIN);
    }
    /* Direction is stable for one complete pulse period before the first
     * falling STEP edge. */
    timer_schedule_ticks(pps_to_period_ticks(initialPps));
}

static void set_direction_locked(int8_t direction)
{
    s_direction = direction;
    if (direction > 0) {
        DL_GPIO_setPins(STEPPER_PORT, STEPPER_DIR_PIN);
    } else {
        DL_GPIO_clearPins(STEPPER_PORT, STEPPER_DIR_PIN);
    }
}

static void begin_tracking_locked(int32_t targetSteps)
{
    const int8_t direction = (targetSteps > s_position_steps) ? 1 : -1;

    s_target_steps = targetSteps;
    s_motion_kind = MOTION_TRACK;
    s_pulse_rate_pps = STEPPER_START_PPS;
    s_move_done = 0U;
    s_moving = 1U;
    s_step_low = 0U;
    set_direction_locked(direction);
    /* Give DIR one complete start-speed period to settle before STEP falls. */
    timer_schedule_ticks(pps_to_period_ticks(STEPPER_START_PPS));
}

static void begin_manual_jog_locked(
    int8_t direction,
    uint16_t pulseRatePps,
    int32_t minSteps,
    int32_t maxSteps)
{
    s_manual_jog_min_steps = minSteps;
    s_manual_jog_max_steps = maxSteps;
    s_motion_kind = MOTION_MANUAL_JOG;
    s_pulse_rate_pps = pulseRatePps;
    s_move_done = 0U;
    s_moving = 1U;
    s_step_low = 0U;
    set_direction_locked(direction);
    /* The fixed low rate gives DIR a complete period to settle before the
     * first pulse; manual leveling never ramps to tracking speed. */
    timer_schedule_ticks(pps_to_period_ticks(pulseRatePps));
}

static void finish_motion_from_isr(void)
{
    s_moving = 0U;
    s_move_done = 1U;
    s_step_low = 0U;
    DL_TimerG_stopCounter(STEPPER_TIMER);
    DL_GPIO_setPins(STEPPER_PORT, STEPPER_STP_PIN);
}

/* This is called only by TIMG0_IRQHandler below. Reading IIDX acknowledges
 * the interrupt on MSPM0; do not clear the status register separately. */
static void stepper_timer_isr(void)
{
    int32_t targetDelta;
    uint32_t trackingStepsLeft;
    uint16_t periodTicks;

    if (DL_TimerG_getPendingInterrupt(STEPPER_TIMER) !=
        DL_TIMERG_IIDX_ZERO) {
        return;
    }
    if (s_moving == 0U) {
        DL_TimerG_stopCounter(STEPPER_TIMER);
        DL_GPIO_setPins(STEPPER_PORT, STEPPER_STP_PIN);
        return;
    }

    if (s_step_low != 0U) {
        /* End of the 20-us active-low input pulse. */
        DL_GPIO_setPins(STEPPER_PORT, STEPPER_STP_PIN);
        s_step_low = 0U;
        periodTicks = pps_to_period_ticks(s_pulse_rate_pps);
        timer_schedule_ticks((uint16_t) (periodTicks -
                                         STEPPER_PULSE_LOW_TICKS));
        return;
    }

    if (s_motion_kind == MOTION_TRACK) {
        targetDelta = s_target_steps - s_position_steps;
        if (targetDelta == 0) {
            finish_motion_from_isr();
            return;
        }

        /*
         * A camera update may move the target across the current position.
         * Restart at the conservative start rate and allow a full DIR setup
         * period instead of emitting another pulse in the old direction.
         * The target itself is slew-limited by the higher-level controller.
         */
        if (((targetDelta > 0) && (s_direction < 0)) ||
            ((targetDelta < 0) && (s_direction > 0))) {
            s_pulse_rate_pps = STEPPER_START_PPS;
            set_direction_locked((targetDelta > 0) ? 1 : -1);
            timer_schedule_ticks(pps_to_period_ticks(STEPPER_START_PPS));
            return;
        }

        /* These checks are redundant with target validation but ensure that
         * no asynchronous target change can pulse beyond a soft boundary. */
        if (((s_direction > 0) &&
             (s_position_steps >= s_travel_max_steps)) ||
            ((s_direction < 0) &&
             (s_position_steps <= s_travel_min_steps))) {
            s_target_steps = s_position_steps;
            finish_motion_from_isr();
            return;
        }

        DL_GPIO_clearPins(STEPPER_PORT, STEPPER_STP_PIN);
        s_step_low = 1U;
        s_position_steps += s_direction;
        targetDelta = s_target_steps - s_position_steps;
        trackingStepsLeft = (targetDelta >= 0) ?
            (uint32_t) targetDelta :
            (uint32_t) (-(targetDelta + 1)) + 1U;
        if (trackingStepsLeft > 0xFFFFU) {
            trackingStepsLeft = 0xFFFFU;
        }
        s_pulse_rate_pps = next_pulse_rate(s_pulse_rate_pps,
                                            (uint16_t) trackingStepsLeft);
        timer_schedule_ticks(STEPPER_PULSE_LOW_TICKS);
        return;
    }
    if (s_motion_kind == MOTION_MANUAL_JOG) {
        if (((s_direction > 0) &&
             (s_position_steps >= s_manual_jog_max_steps)) ||
            ((s_direction < 0) &&
             (s_position_steps <= s_manual_jog_min_steps))) {
            s_target_steps = s_position_steps;
            s_fault = STEPPER_BEAM_FAULT_TRAVEL_LIMIT;
            finish_motion_from_isr();
            return;
        }
        DL_GPIO_clearPins(STEPPER_PORT, STEPPER_STP_PIN);
        s_step_low = 1U;
        s_position_steps += s_direction;
        timer_schedule_ticks(STEPPER_PULSE_LOW_TICKS);
        return;
    }
    if (s_remaining_steps == 0) {
        finish_motion_from_isr();
        return;
    }

    DL_GPIO_clearPins(STEPPER_PORT, STEPPER_STP_PIN);
    s_step_low = 1U;
    s_remaining_steps--;
    s_position_steps += s_direction;
    s_pulse_rate_pps = next_pulse_rate(s_pulse_rate_pps,
                                       (uint16_t) s_remaining_steps);
    timer_schedule_ticks(STEPPER_PULSE_LOW_TICKS);
}

/* Strong vector symbol: this overrides the weak default handler in TI's
 * startup file. It is safer than changing the vector table at run time. */
void TIMG0_IRQHandler(void)
{
    stepper_timer_isr();
}

void StepperBeam_init(void)
{
    DL_GPIO_enablePower(STEPPER_PORT);
    delay_cycles(16U);
    DL_GPIO_initDigitalOutput(STEPPER_STP_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_EN_IOMUX);
    outputs_idle_high();
    DL_GPIO_enableOutput(STEPPER_PORT, STEPPER_ALL_PINS);

    s_position_steps = 0;
    s_target_steps = 0;
    s_travel_min_steps = 0;
    s_travel_max_steps = 0;
    s_remaining_steps = 0;
    s_pulse_rate_pps = 0U;
    s_armed = 0U;
    s_moving = 0U;
    s_step_low = 0U;
    s_move_done = 0U;
    s_travel_limits_configured = 0U;
    s_direction = 1;
    s_motion_kind = MOTION_NONE;
    s_target_steps = s_position_steps;
    s_home_state = STEPPER_BEAM_HOME_UNREFERENCED;
    s_fault = STEPPER_BEAM_FAULT_NONE;
    s_timer_ready = 0U;
    s_manual_jog_min_steps = 0;
    s_manual_jog_max_steps = 0;
}

void StepperBeam_stop(void)
{
    const uint32_t primask = irq_lock();

    s_moving = 0U;
    s_remaining_steps = 0;
    s_step_low = 0U;
    /* The timer is stopped below, so status telemetry must not retain the
     * last motion rate.  A non-zero value here would falsely claim STEP
     * pulses are still being emitted after a LOST/stop safety transition. */
    s_pulse_rate_pps = 0U;
    s_motion_kind = MOTION_NONE;
    s_target_steps = s_position_steps;
    if (s_timer_ready != 0U) {
        DL_TimerG_stopCounter(STEPPER_TIMER);
    }
    outputs_idle_high();
    irq_restore(primask);
}

void StepperBeam_setArmed(bool armed)
{
    if (!armed) {
        StepperBeam_stop();
        s_armed = 0U;
        return;
    }
    if (s_fault == STEPPER_BEAM_FAULT_NONE) {
        prepare_pulse_timer();
        /* X42S has common-anode optocoupler inputs: EN is active low.
         * Keep it inactive/high until the user explicitly arms a test. */
        DL_GPIO_clearPins(STEPPER_PORT, STEPPER_EN_PIN);
        s_armed = 1U;
    }
}

bool StepperBeam_queueJog(int16_t relativeSteps)
{
    const uint32_t primask = irq_lock();
    bool accepted = false;
    const int32_t requestedTarget = s_position_steps + relativeSteps;

    if ((relativeSteps != 0) && (s_armed != 0U) && (s_moving == 0U) &&
        (s_fault == STEPPER_BEAM_FAULT_NONE) &&
        ((s_travel_limits_configured == 0U) ||
         ((requestedTarget >= s_travel_min_steps) &&
          (requestedTarget <= s_travel_max_steps)))) {
        begin_motion_locked(relativeSteps, MOTION_JOG, STEPPER_START_PPS);
        accepted = true;
    }
    irq_restore(primask);
    return accepted;
}

bool StepperBeam_queueBoundedJog(
    int16_t relativeSteps,
    int32_t minSteps,
    int32_t maxSteps)
{
    const uint32_t primask = irq_lock();
    bool accepted = false;
    const int32_t requestedTarget = s_position_steps + relativeSteps;

    if ((minSteps <= maxSteps) && (relativeSteps != 0) &&
        (s_armed != 0U) && (s_moving == 0U) &&
        (s_fault == STEPPER_BEAM_FAULT_NONE) &&
        (requestedTarget >= minSteps) && (requestedTarget <= maxSteps)) {
        begin_motion_locked(relativeSteps, MOTION_JOG, STEPPER_START_PPS);
        accepted = true;
    }
    irq_restore(primask);
    return accepted;
}

bool StepperBeam_startManualJog(
    int8_t direction,
    uint16_t pulseRatePps,
    int32_t minSteps,
    int32_t maxSteps)
{
    const uint32_t primask = irq_lock();
    bool accepted = false;

    if (((direction == 1) || (direction == -1)) &&
        (pulseRatePps != 0U) && (minSteps < maxSteps) &&
        (s_armed != 0U) &&
        (s_fault == STEPPER_BEAM_FAULT_NONE) &&
        (s_position_steps >= minSteps) &&
        (s_position_steps <= maxSteps)) {
        if (s_moving == 0U) {
            begin_manual_jog_locked(direction, pulseRatePps,
                                    minSteps, maxSteps);
            accepted = true;
        } else if ((s_motion_kind == MOTION_MANUAL_JOG) &&
                   (s_direction == direction)) {
            /* Keep the current low-rate jog alive without restarting it. */
            accepted = true;
        }
    }
    irq_restore(primask);
    return accepted;
}

bool StepperBeam_acceptManualReference(void)
{
    const uint32_t primask = irq_lock();
    bool accepted = false;

    if ((s_armed != 0U) &&
        (s_moving == 0U) &&
        (s_fault == STEPPER_BEAM_FAULT_NONE)) {
        /* Keep the raw pulse coordinate.  It is the caller's P_level. */
        s_target_steps = s_position_steps;
        s_travel_min_steps = 0;
        s_travel_max_steps = 0;
        s_travel_limits_configured = 0U;
        s_home_state = STEPPER_BEAM_HOME_READY;
        accepted = true;
    }
    irq_restore(primask);
    return accepted;
}

bool StepperBeam_configureTravelLimits(int32_t minSteps, int32_t maxSteps)
{
    const uint32_t primask = irq_lock();
    bool accepted = false;

    if ((s_moving == 0U) &&
        (s_fault == STEPPER_BEAM_FAULT_NONE) &&
        (s_home_state == STEPPER_BEAM_HOME_READY) &&
        (minSteps < maxSteps) &&
        (s_position_steps >= minSteps) &&
        (s_position_steps <= maxSteps)) {
        s_travel_min_steps = minSteps;
        s_travel_max_steps = maxSteps;
        s_travel_limits_configured = 1U;
        accepted = true;
    }
    irq_restore(primask);
    return accepted;
}

bool StepperBeam_setTargetSteps(int32_t targetSteps)
{
    const uint32_t primask = irq_lock();
    bool accepted = false;

    if ((s_armed != 0U) &&
        (s_fault == STEPPER_BEAM_FAULT_NONE) &&
        (s_home_state == STEPPER_BEAM_HOME_READY) &&
        (s_travel_limits_configured != 0U) &&
        (targetSteps >= s_travel_min_steps) &&
        (targetSteps <= s_travel_max_steps) &&
        ((s_moving == 0U) || (s_motion_kind == MOTION_TRACK))) {
        if (targetSteps == s_position_steps) {
            s_target_steps = targetSteps;
            accepted = true;
        } else if ((s_moving != 0U) &&
                   (s_motion_kind == MOTION_TRACK)) {
            s_target_steps = targetSteps;
            accepted = true;
        } else {
            begin_tracking_locked(targetSteps);
            accepted = true;
        }
    }
    irq_restore(primask);
    return accepted;
}

bool StepperBeam_beginHome(void)
{
    return false;
}

void StepperBeam_service(void)
{
    uint32_t primask;

    if (s_move_done == 0U) {
        return;
    }
    primask = irq_lock();
    if (s_move_done == 0U) {
        irq_restore(primask);
        return;
    }
    s_move_done = 0U;
    s_motion_kind = MOTION_NONE;
    irq_restore(primask);
}

bool StepperBeam_clearFault(void)
{
    if (s_moving != 0U) {
        return false;
    }
    s_fault = STEPPER_BEAM_FAULT_NONE;
    s_home_state = STEPPER_BEAM_HOME_UNREFERENCED;
    s_position_steps = 0;
    s_target_steps = 0;
    s_travel_min_steps = 0;
    s_travel_max_steps = 0;
    s_travel_limits_configured = 0U;
    s_armed = 0U;
    return true;
}

bool StepperBeam_markCurrentPositionZero(void)
{
    const uint32_t primask = irq_lock();
    bool accepted = false;

    if (s_moving == 0U) {
        s_position_steps = 0;
        s_target_steps = 0;
        s_travel_min_steps = 0;
        s_travel_max_steps = 0;
        s_travel_limits_configured = 0U;
        accepted = true;
    }
    irq_restore(primask);
    return accepted;
}

void StepperBeam_getStatus(StepperBeamStatus *status)
{
    uint32_t primask;

    if (status == 0) {
        return;
    }
    primask = irq_lock();
    status->positionSteps = s_position_steps;
    status->targetSteps = s_target_steps;
    status->travelMinSteps = s_travel_min_steps;
    status->travelMaxSteps = s_travel_max_steps;
    status->pulseRatePps = s_pulse_rate_pps;
    status->armed = s_armed;
    status->moving = s_moving;
    status->homeInputActive = 0U;
    status->travelLimitsConfigured = s_travel_limits_configured;
    status->homeState = s_home_state;
    status->fault = s_fault;
    irq_restore(primask);
}
