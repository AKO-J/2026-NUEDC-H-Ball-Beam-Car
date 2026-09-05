#ifndef BEAM_CALIBRATION_H
#define BEAM_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Sole conversion table for the first X42S/white-beam calibration.
 * Angles are white-beam angles in millidegrees, never X42S shaft angles.
 */
enum {
    BEAM_CAL_MIN_ANGLE_MDEG = -2221,
    BEAM_CAL_MAX_ANGLE_MDEG = 1777,
    BEAM_CAL_MIN_POS = -132,
    BEAM_CAL_MAX_POS = 132,
};

typedef struct {
    int16_t angleLowMdeg;
    int16_t angleHighMdeg;
    int16_t posLow;
    int16_t posHigh;
    int16_t targetPos;
} BeamCalibrationInterpolation;

/*
 * Piecewise-linear white-beam-angle -> X42S POS conversion.
 *
 * Returns false instead of extrapolating outside the measured range.
 * When true is returned, "result" also records the two calibration points
 * actually used, making every commanded POS auditable on the OLED/debugger.
 */
bool BeamCalibration_angleToPos(
    int16_t beamAngleMdeg,
    BeamCalibrationInterpolation *result);

#endif
