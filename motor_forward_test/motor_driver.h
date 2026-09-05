#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/*
 * TB6612 motor channels on the current LP-MSPM0G3507 wiring.
 * Channel A and B are electrical channels.  Map them to left/right wheels in
 * the application only after the mechanical installation is fixed.
 */
typedef enum {
    MOTOR_CHANNEL_A,
    MOTOR_CHANNEL_B,
} MotorChannel;

typedef enum {
    MOTOR_DIRECTION_FORWARD,
    MOTOR_DIRECTION_REVERSE,
} MotorDirection;

/* Latest commanded continuous PWM duties, exported for telemetry only. */
extern volatile uint8_t g_motor_pwm_a_duty;
extern volatile uint8_t g_motor_pwm_b_duty;
extern volatile uint8_t g_motor_pwm_continuous;

/* Initialise the two 20-kHz hardware PWM channels and release both bridges. */
void Motor_init(void);

/* Stop one channel or both channels by releasing the TB6612 bridge inputs. */
void Motor_stop(MotorChannel channel);
void Motor_stopAll(void);

/*
 * TB6612 active short-brake for both wheels. Unlike Motor_stopAll(), which
 * releases the bridge, this drives both inputs high while PWM is high so the
 * wheels resist coasting. Intended for brief corner-braking intervals only.
 */
void Motor_brakeAllFor(uint32_t durationMs);

/*
 * Blocking helpers for bring-up and low-speed tests, using hardware PWM.
 * dutyPercent is limited to 0..100.  Each call stops the affected motor(s)
 * automatically before returning.
 */
void Motor_runFor(MotorChannel channel, MotorDirection direction,
                  uint8_t dutyPercent, uint32_t durationMs);
void Motor_runBothFor(MotorDirection directionA, MotorDirection directionB,
                      uint8_t dutyPercent, uint32_t durationMs);

/*
 * Each bridge gets an independent, continuously generated hardware PWM duty.
 * The call blocks only for durationMs so existing control loops retain their
 * timing; it deliberately does NOT stop either motor on return.  The next
 * call updates the duty, and Motor_stop* is the explicit safety stop.  A zero
 * duty fully releases that channel while the other channel may continue.
 */
void Motor_runBothWithDutyFor(MotorDirection directionA,
                              MotorDirection directionB,
                              uint8_t dutyA, uint8_t dutyB,
                              uint32_t durationMs);

/* Non-blocking continuous-duty update for cooperative application loops. */
void Motor_setBothDuty(MotorDirection directionA, MotorDirection directionB,
                       uint8_t dutyA, uint8_t dutyB);

#endif /* MOTOR_DRIVER_H */
