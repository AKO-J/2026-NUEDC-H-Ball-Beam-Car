#ifndef H_TRACK_CONTROLLER_H_
#define H_TRACK_CONTROLLER_H_

#include <stdint.h>

/* Native LF04 state machine.  dhWhiteState is the manufacturer's DH1..DH4
 * representation: physical left to right, white=1 and black=0. */
typedef enum {
    H_TRACK_STATE_LEAVE_START_MARK = 0U,
    H_TRACK_STATE_FOLLOW = 1U,
    H_TRACK_STATE_CONFIRM_A_MARK = 2U,
    H_TRACK_STATE_STOP_APPROACH = 3U,
    H_TRACK_STATE_FINISHED = 4U,
    H_TRACK_STATE_FAULT = 5U,
} HTrackState;

typedef enum {
    H_TRACK_DRIVE_STOP = 0U,
    H_TRACK_DRIVE_CENTER = 1U,
    H_TRACK_DRIVE_LINE_TURN = 2U,
    H_TRACK_DRIVE_HOLD_LAST = 3U,
    H_TRACK_DRIVE_STOP_APPROACH = 4U,
} HTrackDrive;

typedef struct {
    uint8_t centerLeftDuty;
    uint8_t centerRightDuty;
    uint8_t smallInnerDuty;
    uint8_t smallOuterDuty;
    uint8_t bigInnerDuty;
    uint8_t bigOuterDuty;
    uint8_t sharpInnerDuty;
    uint8_t sharpOuterDuty;
    uint8_t approachDuty;
    uint8_t finishDuty;
    uint16_t markerConfirmMs;
    uint32_t lapArmCounts;
    uint32_t stopOffsetCounts;
    uint32_t stopSlowWindowCounts;
} HTrackConfig;

typedef struct {
    HTrackDrive drive;
    int8_t direction;             /* -1 left, +1 right */
    uint8_t innerDuty;
    uint8_t outerDuty;
} HTrackCommand;

typedef struct {
    HTrackConfig config;
    HTrackState state;
    int8_t lastDirection;
    uint8_t lastInnerDuty;
    uint8_t lastOuterDuty;
    uint8_t aMarkerArmed;
    uint16_t markerElapsedMs;
    uint32_t markerDetectedCounts;
    uint32_t stopTargetCounts;
} HTrackController;

void HTrackController_init(HTrackController *controller,
                           const HTrackConfig *config);

/* Allows the application to gate A-marker acceptance (for example, by a
 * completed IMU lap) while leaving the controller responsible for debounce,
 * encoder compensation, and the final low-speed stop. */
void HTrackController_setAMarkerArmed(HTrackController *controller,
                                      uint8_t armed);

/* distanceCounts is the forward average of the two wheel encoders. */
HTrackCommand HTrackController_step(HTrackController *controller,
                                    uint8_t dhWhiteState,
                                    uint32_t distanceCounts,
                                    uint16_t elapsedMs);

#endif /* H_TRACK_CONTROLLER_H_ */
