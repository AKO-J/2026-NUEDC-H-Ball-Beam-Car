#ifndef DIAGNOSTIC_UART_H
#define DIAGNOSTIC_UART_H

#include <stdint.h>

#include "stepper_beam.h"
#include "vision_ball_protocol.h"

typedef struct {
    uint32_t mcuMs;
    uint32_t runId;
    const char *state;
    const char *safetyState;
    uint8_t irRawMask;
    int32_t lineError;
    int32_t lineErrorRate;
    int32_t lineCorrection;
    int32_t baseSpeedRef;
    int32_t baseAccelerationRef;
    int32_t leftSpeedRef;
    int32_t rightSpeedRef;
    int32_t leftSpeed;
    int32_t rightSpeed;
    int32_t leftSpeedError;
    int32_t rightSpeedError;
    uint8_t leftPwm;
    uint8_t rightPwm;
    int32_t leftEncoder;
    int32_t rightEncoder;
    int32_t vehicleSpeed;
    int32_t vehicleAcceleration;
    int32_t imuAccelX;
    int32_t imuAccelY;
    int32_t imuAccelZ;
    int32_t kffMilli;
    int32_t beamFeedforwardMdeg;
    int32_t beamBallPdMdeg;
    int32_t beamTargetMdeg;
    int32_t stepperTarget;
    int32_t stepperPosition;
    uint32_t stepFrequency;
    uint32_t visionFrame;
    int32_t visionAgeMs;
    int32_t ballTargetCmX100;
    int32_t ballErrorCmX100;
    int32_t ballVelocityCmPerSX100;
    uint32_t faultFlags;
} DiagnosticJointTelemetry;

/* XDS110 backchannel UART0, PA10 TX, 115200-8N1. */
void DiagnosticUart_init(void);

/* Emit exactly one headerless CSV line for the host recorder. */
void DiagnosticUart_writeCsv(
    uint32_t pcMs,
    const VisionBallMeasurement *vision,
    int32_t ageMs,
    int16_t targetX,
    int32_t velocityPxPerS,
    int16_t errorPx,
    int16_t commandMdeg,
    const StepperBeamStatus *motor,
    const char *state);

void DiagnosticUart_writeTask4Csv(
    uint32_t pcMs, const VisionBallMeasurement *vision, int32_t ballErrorCmX100,
    int32_t ageMs, int32_t velocityPxPerS, int16_t commandMdeg,
    const StepperBeamStatus *motor, int32_t leftEncoder, int32_t rightEncoder,
    int32_t leftSpeedCountsPerS, int32_t rightSpeedCountsPerS,
    int32_t accelXRaw, int32_t accelYRaw, int32_t accelZRaw,
    uint8_t leftPwm, uint8_t rightPwm, const char *phase,
    const char *safetyState);

void DiagnosticUart_writeJointCsv(const DiagnosticJointTelemetry *telemetry);

#endif
