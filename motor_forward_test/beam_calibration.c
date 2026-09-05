#include "beam_calibration.h"

#include <stddef.h>

typedef struct {
    int16_t angleMdeg;
    int16_t pos;
} CalibrationPoint;

/*
 * Source: X42S_白杆首次标定结果_2026-07-31.md, section 5.
 * Keep this table asymmetric: the measured negative and positive sides have
 * different slopes and must not be replaced with one pulses/degree constant.
 */
static const CalibrationPoint kPoints[] = {
    {-2221, -132},
    {-1333,  -88},
    { -444,  -44},
    {    0,    0},
    {  444,   44},
    { 1110,   88},
    { 1777,  132},
};

static int16_t interpolateRounded(
    int16_t x,
    const CalibrationPoint *low,
    const CalibrationPoint *high)
{
    const int32_t numerator =
        (int32_t) (x - low->angleMdeg) * (high->pos - low->pos);
    const int32_t denominator =
        (int32_t) high->angleMdeg - low->angleMdeg;

    /* All table segments are ascending, so numerator is non-negative. */
    return (int16_t) (low->pos +
        (numerator + denominator / 2) / denominator);
}

bool BeamCalibration_angleToPos(
    int16_t beamAngleMdeg,
    BeamCalibrationInterpolation *result)
{
    size_t i;

    if ((result == NULL) ||
        (beamAngleMdeg < BEAM_CAL_MIN_ANGLE_MDEG) ||
        (beamAngleMdeg > BEAM_CAL_MAX_ANGLE_MDEG)) {
        return false;
    }

    for (i = 0U; i + 1U < sizeof(kPoints) / sizeof(kPoints[0]); ++i) {
        const CalibrationPoint *low = &kPoints[i];
        const CalibrationPoint *high = &kPoints[i + 1U];

        if (beamAngleMdeg <= high->angleMdeg) {
            result->angleLowMdeg = low->angleMdeg;
            result->angleHighMdeg = high->angleMdeg;
            result->posLow = low->pos;
            result->posHigh = high->pos;
            result->targetPos =
                interpolateRounded(beamAngleMdeg, low, high);
            return true;
        }
    }
    return false;
}
