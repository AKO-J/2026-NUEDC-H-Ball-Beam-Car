#ifndef H_TRACK_FINISH_H_
#define H_TRACK_FINISH_H_

#include <stdint.h>

/*
 * Distance-only finish detector for a course whose A cross-line cannot be
 * read reliably. Distance units are chosen by the caller; the task-2
 * application uses 0.01 cm so left/right encoder scale factors can be applied
 * before the two wheel distances are averaged.
 */
typedef struct {
    uint32_t slowStartDistance;
    uint32_t stopDistance;
} HTrackFinishConfig;

typedef struct {
    HTrackFinishConfig config;
    uint8_t armed;
    uint8_t reached;
} HTrackFinishDetector;

void HTrackFinishDetector_init(HTrackFinishDetector *detector,
                               const HTrackFinishConfig *config);

/* Once stopDistance is reached, the result stays latched until reinitialised.
 * armed becomes 1 at slowStartDistance so the application can reduce speed
 * while continuing to follow the final part of the line. */
uint8_t HTrackFinishDetector_step(HTrackFinishDetector *detector,
                                  uint32_t distance);

#endif /* H_TRACK_FINISH_H_ */
