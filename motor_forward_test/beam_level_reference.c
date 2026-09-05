#include "beam_level_reference.h"

#include "beam_calibration.h"

bool BeamLevelReference_makeTarget(
    int32_t levelReferencePos,
    int16_t relativeControlPos,
    int32_t *motorTargetPos)
{
    if ((motorTargetPos == 0) ||
        (relativeControlPos < BEAM_CAL_MIN_POS) ||
        (relativeControlPos > BEAM_CAL_MAX_POS)) {
        return false;
    }
    *motorTargetPos = levelReferencePos + (int32_t) relativeControlPos;
    return true;
}
