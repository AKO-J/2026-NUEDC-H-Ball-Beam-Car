#include "h_track_finish.h"

void HTrackFinishDetector_init(HTrackFinishDetector *detector,
                               const HTrackFinishConfig *config)
{
    detector->config = *config;
    detector->armed = 0U;
    detector->reached = 0U;
}

uint8_t HTrackFinishDetector_step(HTrackFinishDetector *detector,
                                  uint32_t distance)
{
    if (detector->reached != 0U) {
        return 1U;
    }

    if (distance >= detector->config.slowStartDistance) {
        detector->armed = 1U;
    }

    if (distance >= detector->config.stopDistance) {
        detector->reached = 1U;
    }
    return detector->reached;
}
