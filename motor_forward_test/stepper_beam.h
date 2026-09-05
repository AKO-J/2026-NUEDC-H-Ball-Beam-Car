#ifndef STEPPER_BEAM_H
#define STEPPER_BEAM_H

/*
 * X42S closed-loop stepper pulse interface.
 *
 * Wiring for the first, isolated test image:
 *   X42S COM -> MSPM0 3V3 (common-anode optocoupler input)
 *   X42S STP -> PB13, DIR -> PB12, EN -> PB6
 *
 * The current ball-beam mechanism does not use a mechanical home switch.
 * Each power-up is manually leveled and accepted as the software reference.
 *
 * X42S V+ / Gnd are the 12-V motor supply only. COM is deliberately not
 * tied to the motor power Gnd. The input is active when the MSPM0 output
 * sinks it low, so the idle level of STP, DIR and EN is high.
 */

#include <stdbool.h>
#include <stdint.h>

enum {
    /* Deliberately visible empty-shaft test move: 100 pulses is about
     * 11.25 degrees at 16 microsteps on a 1.8-degree motor. */
    STEPPER_BEAM_JOG_STEPS = 100,
};

typedef enum {
    STEPPER_BEAM_FAULT_NONE = 0,
    STEPPER_BEAM_FAULT_HOME_TIMEOUT,
    STEPPER_BEAM_FAULT_HOME_STUCK,
    /* A guarded manual-level jog reached its caller-supplied safe bound. */
    STEPPER_BEAM_FAULT_TRAVEL_LIMIT,
} StepperBeamFault;

typedef enum {
    STEPPER_BEAM_HOME_UNREFERENCED = 0,
    STEPPER_BEAM_HOME_SEEKING,
    STEPPER_BEAM_HOME_BACKING_OFF,
    STEPPER_BEAM_HOME_READY,
} StepperBeamHomeState;

typedef struct {
    int32_t positionSteps;
    int32_t targetSteps;
    int32_t travelMinSteps;
    int32_t travelMaxSteps;
    uint16_t pulseRatePps;
    uint8_t armed;
    uint8_t moving;
    uint8_t homeInputActive;
    uint8_t travelLimitsConfigured;
    StepperBeamHomeState homeState;
    StepperBeamFault fault;
} StepperBeamStatus;

/* Configure PB13/PB12/PB6 and TIMG0. Outputs are safe/high at return. */
void StepperBeam_init(void);

/* Disarming cancels a queued move and drives STP/DIR/EN to their safe high
 * idle levels. It does not remove the X42S 12-V supply or holding torque. */
void StepperBeam_setArmed(bool armed);

/* Issue a finite relative pulse move. This is deliberately usable before
 * the manual reference is accepted. Returns false if disarmed,
 * busy, faulted, or the request is zero. */
bool StepperBeam_queueJog(int16_t relativeSteps);

/* A finite pre-reference move with an explicit absolute software envelope.
 * It is for a manually-confirmed bootstrap location only (for example, a
 * lower mechanical stop before the level reference exists).  Unlike the
 * legacy unrestricted pre-reference jog, the requested endpoint must be
 * inside [minSteps, maxSteps]. */
bool StepperBeam_queueBoundedJog(
    int16_t relativeSteps,
    int32_t minSteps,
    int32_t maxSteps);

/* Start or keep a manual level-adjust jog running at a caller-selected low
 * fixed rate.  This mode is intentionally separate from closed-loop tracking:
 * it never changes the target used by the controller and it stops itself at
 * the supplied inclusive bounds.  Passing the opposite direction is refused
 * while moving, so callers must stop before reversing. */
bool StepperBeam_startManualJog(
    int8_t direction,
    uint16_t pulseRatePps,
    int32_t minSteps,
    int32_t maxSteps);

/* Accept the current stopped position as the manual beam reference. This is
 * used when the mechanism has been leveled by jogging and intentionally has
 * no mechanical home switch. It preserves the current software pulse
 * coordinate (the caller records it as P_level) and marks the reference
 * ready; conservative travel limits must still be configured next. */
bool StepperBeam_acceptManualReference(void);

/* Configure the allowed absolute coordinate range for closed-loop tracking.
 * This is accepted only while stopped and after the manual level reference. */
bool StepperBeam_configureTravelLimits(int32_t minSteps, int32_t maxSteps);

/* Continuously track an absolute pulse coordinate. A new target may replace
 * the previous target while tracking is active; the pulse ISR will follow the
 * latest value. Tracking is refused until the manual reference and travel-limit
 * configuration have both completed. Targets outside the configured range
 * are rejected rather than clamped. */
bool StepperBeam_setTargetSteps(int32_t targetSteps);

/* Mechanical-switch homing is disabled for this mechanism. This compatibility
 * entry point always returns false and never starts the motor. */
bool StepperBeam_beginHome(void);

/* Called from the foreground loop to retire completed finite moves. */
void StepperBeam_service(void);

/* Immediately cancels all future STEP pulses. */
void StepperBeam_stop(void);

/* Fault acknowledgement: only works after motion has stopped. */
bool StepperBeam_clearFault(void);

/* Bench-test coordinate reset. This changes only the software pulse counter;
 * it does not move the X42S. It is allowed only while stopped and invalidates
 * any previously configured closed-loop travel limits. */
bool StepperBeam_markCurrentPositionZero(void);

void StepperBeam_getStatus(StepperBeamStatus *status);

#endif
