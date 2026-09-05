#ifndef BALL_BEAM_CONTROLLER_H
#define BALL_BEAM_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "beam_calibration.h"

/*
 * Pixel-domain PD controller with an alpha-beta visual state estimator.
 *
 * The gains are deliberately integer and easy to tune. They do not claim a
 * physical pixel-to-centimetre calibration. The only physical conversion is
 * the measured white-beam angle -> POS table in beam_calibration.c.
 */
enum {
    BALL_BEAM_CAMERA_CENTER_X_PX = 160,
    /* Tuned for the task-3 motion; task code adds a ±800 mdeg safety cap. */
    BALL_BEAM_KP_MDEG_PER_PX = 11,
    BALL_BEAM_KD_NUMERATOR = 1,
    BALL_BEAM_KD_DENOMINATOR = 1,
    BALL_BEAM_ERROR_DEADBAND_PX = 2,
};

typedef struct {
    int16_t targetXOffsetPx;
    int16_t previousXOffsetPx;
    int16_t estimatedXOffsetPx;
    int32_t filteredVelocityPxPerS;
    uint32_t previousSampleMs;
    uint8_t initialized;
} BallBeamController;

typedef struct {
    int16_t errorPx;
    int32_t velocityPxPerS;
    /* One visual-frame look-ahead from alpha-beta position/velocity state. */
    int16_t predictedXOffsetPx;
    int16_t requestedAngleMdeg;
    BeamCalibrationInterpolation calibration;
} BallBeamCommand;

void BallBeamController_init(
    BallBeamController *controller,
    int16_t targetXPixel);

/*
 * Converts one fresh camera sample into a calibrated POS command.
 * controlSign must be +1 or -1 and is verified during the no-ball direction
 * test. Returns false only for bad arguments/calibration failure.
 */
bool BallBeamController_update(
    BallBeamController *controller,
    int16_t measuredXOffsetPx,
    uint32_t sampleTimeMs,
    int8_t controlSign,
    BallBeamCommand *command);

#endif
