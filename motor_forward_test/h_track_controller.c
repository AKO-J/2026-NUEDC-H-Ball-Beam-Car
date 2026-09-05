#include "h_track_controller.h"

enum {
    LF04_STATE_MARK_OR_CROSS = 0x0U,
    LF04_STATE_LEFT_90_A = 0x1U,
    LF04_STATE_LEFT_90_B = 0x3U,
    LF04_STATE_LEFT_BIG = 0x7U,
    LF04_STATE_RIGHT_90_A = 0x8U,
    LF04_STATE_STRAIGHT = 0x9U,
    LF04_STATE_LEFT_SMALL = 0xBU,
    LF04_STATE_RIGHT_90_B = 0xCU,
    LF04_STATE_RIGHT_SMALL = 0xDU,
    LF04_STATE_RIGHT_BIG = 0xEU,
    LF04_STATE_LOST = 0xFU,
};

static uint16_t add_elapsed(uint16_t value, uint16_t elapsedMs)
{
    return (value > (uint16_t) (65535U - elapsedMs)) ? 65535U :
           (uint16_t) (value + elapsedMs);
}

static uint32_t add_counts(uint32_t value, uint32_t increment)
{
    return (value > (UINT32_MAX - increment)) ? UINT32_MAX :
           value + increment;
}

static HTrackCommand stop_command(void)
{
    HTrackCommand command = {0};
    command.drive = H_TRACK_DRIVE_STOP;
    return command;
}

static HTrackCommand center_command(const HTrackController *controller)
{
    HTrackCommand command = {0};
    command.drive = H_TRACK_DRIVE_CENTER;
    command.innerDuty = controller->config.centerLeftDuty;
    command.outerDuty = controller->config.centerRightDuty;
    return command;
}

static HTrackCommand turn_command(HTrackController *controller,
                                  int8_t direction, uint8_t innerDuty,
                                  uint8_t outerDuty)
{
    HTrackCommand command = {0};

    command.drive = H_TRACK_DRIVE_LINE_TURN;
    command.direction = direction;
    command.innerDuty = innerDuty;
    command.outerDuty = outerDuty;
    controller->lastDirection = direction;
    controller->lastInnerDuty = innerDuty;
    controller->lastOuterDuty = outerDuty;
    return command;
}

static HTrackCommand hold_last_command(const HTrackController *controller)
{
    if (controller->lastDirection == 0) {
        return center_command(controller);
    }

    {
        HTrackCommand command = {0};
        command.drive = H_TRACK_DRIVE_HOLD_LAST;
        command.direction = controller->lastDirection;
        command.innerDuty = controller->lastInnerDuty;
        command.outerDuty = controller->lastOuterDuty;
        return command;
    }
}

static HTrackCommand follow_command(HTrackController *controller,
                                    uint8_t state)
{
    switch (state & 0x0FU) {
    case LF04_STATE_STRAIGHT:
        return center_command(controller);
    case LF04_STATE_LEFT_SMALL:
        return turn_command(controller, -1,
                            controller->config.smallInnerDuty,
                            controller->config.smallOuterDuty);
    case LF04_STATE_RIGHT_SMALL:
        return turn_command(controller, 1,
                            controller->config.smallInnerDuty,
                            controller->config.smallOuterDuty);
    case LF04_STATE_LEFT_BIG:
        return turn_command(controller, -1,
                            controller->config.bigInnerDuty,
                            controller->config.bigOuterDuty);
    case LF04_STATE_RIGHT_BIG:
        return turn_command(controller, 1,
                            controller->config.bigInnerDuty,
                            controller->config.bigOuterDuty);
    case LF04_STATE_LEFT_90_A:
    case LF04_STATE_LEFT_90_B:
        return turn_command(controller, -1,
                            controller->config.sharpInnerDuty,
                            controller->config.sharpOuterDuty);
    case LF04_STATE_RIGHT_90_A:
    case LF04_STATE_RIGHT_90_B:
        return turn_command(controller, 1,
                            controller->config.sharpInnerDuty,
                            controller->config.sharpOuterDuty);
    case LF04_STATE_LOST:
        return hold_last_command(controller);
    default:
        /* Unknown combinations can occur on a tape edge.  Retain the last
         * safe turn instead of inventing a route choice. */
        return hold_last_command(controller);
    }
}

void HTrackController_init(HTrackController *controller,
                           const HTrackConfig *config)
{
    controller->config = *config;
    controller->state = H_TRACK_STATE_LEAVE_START_MARK;
    controller->lastDirection = 0;
    controller->lastInnerDuty = controller->config.centerLeftDuty;
    controller->lastOuterDuty = controller->config.centerRightDuty;
    /* Preserve the standalone controller's historical distance-only default. */
    controller->aMarkerArmed = 1U;
    controller->markerElapsedMs = 0U;
    controller->markerDetectedCounts = 0U;
    controller->stopTargetCounts = 0U;
}

void HTrackController_setAMarkerArmed(HTrackController *controller,
                                      uint8_t armed)
{
    controller->aMarkerArmed = (armed != 0U) ? 1U : 0U;
}

HTrackCommand HTrackController_step(HTrackController *controller,
                                    uint8_t dhWhiteState,
                                    uint32_t distanceCounts,
                                    uint16_t elapsedMs)
{
    const uint8_t state = dhWhiteState & 0x0FU;

    if ((controller->state == H_TRACK_STATE_FINISHED) ||
        (controller->state == H_TRACK_STATE_FAULT)) {
        return stop_command();
    }

    if (controller->state == H_TRACK_STATE_LEAVE_START_MARK) {
        /* A starts on a transverse black line (0000).  Keep moving through
         * it, but do not allow it to satisfy the one-lap marker condition. */
        if (state != LF04_STATE_MARK_OR_CROSS) {
            controller->state = H_TRACK_STATE_FOLLOW;
        }
        return follow_command(controller, state == LF04_STATE_MARK_OR_CROSS ?
                              LF04_STATE_STRAIGHT : state);
    }

    if (controller->state == H_TRACK_STATE_FOLLOW) {
        if ((controller->aMarkerArmed != 0U) &&
            (distanceCounts >= controller->config.lapArmCounts) &&
            (state == LF04_STATE_MARK_OR_CROSS)) {
            controller->state = H_TRACK_STATE_CONFIRM_A_MARK;
            controller->markerElapsedMs = elapsedMs;
            return hold_last_command(controller);
        }
        return follow_command(controller, state);
    }

    if (controller->state == H_TRACK_STATE_CONFIRM_A_MARK) {
        if (state != LF04_STATE_MARK_OR_CROSS) {
            controller->state = H_TRACK_STATE_FOLLOW;
            controller->markerElapsedMs = 0U;
            return follow_command(controller, state);
        }
        controller->markerElapsedMs =
            add_elapsed(controller->markerElapsedMs, elapsedMs);
        if (controller->markerElapsedMs < controller->config.markerConfirmMs) {
            return hold_last_command(controller);
        }
        controller->markerDetectedCounts = distanceCounts;
        controller->stopTargetCounts = add_counts(
            distanceCounts, controller->config.stopOffsetCounts);
        controller->state = H_TRACK_STATE_STOP_APPROACH;
    }

    if (controller->state == H_TRACK_STATE_STOP_APPROACH) {
        HTrackCommand command = {0};
        uint32_t remaining;

        if (distanceCounts >= controller->stopTargetCounts) {
            controller->state = H_TRACK_STATE_FINISHED;
            return stop_command();
        }
        remaining = controller->stopTargetCounts - distanceCounts;
        command.drive = H_TRACK_DRIVE_STOP_APPROACH;
        command.innerDuty =
            (remaining <= controller->config.stopSlowWindowCounts) ?
            controller->config.finishDuty : controller->config.approachDuty;
        command.outerDuty = command.innerDuty;
        return command;
    }

    controller->state = H_TRACK_STATE_FAULT;
    return stop_command();
}
