#ifndef BEAM_LEVEL_REFERENCE_H
#define BEAM_LEVEL_REFERENCE_H

#include <stdbool.h>
#include <stdint.h>

/* Compose the final motor-coordinate target from the manually confirmed
 * level coordinate and a relative POS returned by the beam calibration table.
 * The relative control coordinate is deliberately restricted to the only
 * calibrated range, [-132, +132]. */
bool BeamLevelReference_makeTarget(
    int32_t levelReferencePos,
    int16_t relativeControlPos,
    int32_t *motorTargetPos);

#endif
