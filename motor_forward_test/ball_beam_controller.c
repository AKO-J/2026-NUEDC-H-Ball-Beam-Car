#include "ball_beam_controller.h"

#include <stddef.h>

/* Alpha-beta gains use only image-position innovation.  They deliberately do
 * not take a second difference of YOLO boxes as a measured acceleration. */
#define ALPHA_NUMERATOR                3
#define ALPHA_DENOMINATOR              4
#define BETA_NUMERATOR                 1
#define BETA_DENOMINATOR               8
#define VISION_LOOKAHEAD_MS            50U

static int32_t clamp32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

void BallBeamController_init(
    BallBeamController *controller,
    int16_t targetXPixel)
{
    if (controller == NULL) {
        return;
    }
    controller->targetXOffsetPx =
        (int16_t) (targetXPixel - BALL_BEAM_CAMERA_CENTER_X_PX);
    controller->previousXOffsetPx = 0;
    controller->estimatedXOffsetPx = 0;
    controller->filteredVelocityPxPerS = 0;
    controller->previousSampleMs = 0U;
    controller->initialized = 0U;
}

bool BallBeamController_update(
    BallBeamController *controller,
    int16_t measuredXOffsetPx,
    uint32_t sampleTimeMs,
    int8_t controlSign,
    BallBeamCommand *command)
{
    uint32_t dtMs;
    int32_t predictedX;
    int32_t innovation;
    int32_t angle;
    int16_t error;

    if ((controller == NULL) || (command == NULL) ||
        ((controlSign != 1) && (controlSign != -1))) {
        return false;
    }

    error = (int16_t)
        (controller->targetXOffsetPx - measuredXOffsetPx);
    if (controller->initialized != 0U) {
        dtMs = sampleTimeMs - controller->previousSampleMs;
        if ((dtMs > 0U) && (dtMs <= 250U)) {
            predictedX = (int32_t) controller->estimatedXOffsetPx +
                controller->filteredVelocityPxPerS * (int32_t) dtMs / 1000;
            innovation = (int32_t) measuredXOffsetPx - predictedX;
            predictedX += innovation * ALPHA_NUMERATOR /
                          ALPHA_DENOMINATOR;
            controller->estimatedXOffsetPx = (int16_t) predictedX;
            controller->filteredVelocityPxPerS =
                controller->filteredVelocityPxPerS +
                innovation * 1000 * BETA_NUMERATOR /
                ((int32_t) dtMs * BETA_DENOMINATOR);
        } else {
            controller->estimatedXOffsetPx = measuredXOffsetPx;
            controller->filteredVelocityPxPerS = 0;
        }
    } else {
        controller->initialized = 1U;
        controller->estimatedXOffsetPx = measuredXOffsetPx;
        controller->filteredVelocityPxPerS = 0;
    }
    controller->previousXOffsetPx = measuredXOffsetPx;
    controller->previousSampleMs = sampleTimeMs;

    if ((error >= -BALL_BEAM_ERROR_DEADBAND_PX) &&
        (error <= BALL_BEAM_ERROR_DEADBAND_PX) &&
        (controller->filteredVelocityPxPerS >= -2) &&
        (controller->filteredVelocityPxPerS <= 2)) {
        angle = 0;
    } else {
        angle =
            (int32_t) BALL_BEAM_KP_MDEG_PER_PX * error -
            ((int32_t) BALL_BEAM_KD_NUMERATOR *
             controller->filteredVelocityPxPerS) /
                BALL_BEAM_KD_DENOMINATOR;
        angle *= controlSign;
    }
    angle = clamp32(
        angle, BEAM_CAL_MIN_ANGLE_MDEG, BEAM_CAL_MAX_ANGLE_MDEG);

    command->errorPx = error;
    command->velocityPxPerS = controller->filteredVelocityPxPerS;
    predictedX = (int32_t) controller->estimatedXOffsetPx +
        controller->filteredVelocityPxPerS *
        (int32_t) VISION_LOOKAHEAD_MS / 1000;
    command->predictedXOffsetPx = (int16_t) clamp32(
        predictedX, -32768, 32767);
    command->requestedAngleMdeg = (int16_t) angle;
    return BeamCalibration_angleToPos(
        command->requestedAngleMdeg, &command->calibration);
}
