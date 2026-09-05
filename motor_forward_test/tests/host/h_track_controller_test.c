#include <assert.h>
#include <stdio.h>

#include "h_track_controller.h"

static HTrackConfig test_config(void)
{
    const HTrackConfig config = {
        .centerLeftDuty = 20U,
        .centerRightDuty = 21U,
        .smallInnerDuty = 16U,
        .smallOuterDuty = 27U,
        .bigInnerDuty = 10U,
        .bigOuterDuty = 35U,
        .sharpInnerDuty = 0U,
        .sharpOuterDuty = 48U,
        .approachDuty = 14U,
        .finishDuty = 8U,
        .markerConfirmMs = 20U,
        .lapArmCounts = 500U,
        .stopOffsetCounts = 80U,
        .stopSlowWindowCounts = 20U,
    };
    return config;
}

static void assert_turn_state(uint8_t dhState, int8_t direction,
                              uint8_t innerDuty, uint8_t outerDuty)
{
    HTrackController controller;
    HTrackCommand command;
    const HTrackConfig config = test_config();

    HTrackController_init(&controller, &config);
    (void) HTrackController_step(&controller, 0x9U, 1U, 4U);
    command = HTrackController_step(&controller, dhState, 2U, 4U);
    assert(command.drive == H_TRACK_DRIVE_LINE_TURN);
    assert(command.direction == direction);
    assert(command.innerDuty == innerDuty);
    assert(command.outerDuty == outerDuty);
}

int main(void)
{
    HTrackController controller;
    HTrackCommand command;
    const HTrackConfig config = test_config();

    HTrackController_init(&controller, &config);

    /* Starting on A's 0000 stripe must not park immediately. */
    command = HTrackController_step(&controller, 0x0U, 0U, 4U);
    assert(controller.state == H_TRACK_STATE_LEAVE_START_MARK);
    assert(command.drive == H_TRACK_DRIVE_CENTER);

    command = HTrackController_step(&controller, 0x9U, 10U, 4U);
    assert(controller.state == H_TRACK_STATE_FOLLOW);
    assert(command.drive == H_TRACK_DRIVE_CENTER);

    command = HTrackController_step(&controller, 0xBU, 100U, 4U);
    assert(command.drive == H_TRACK_DRIVE_LINE_TURN);
    assert(command.direction == -1);

    /* Complete manufacturer state table: 0001/0011 sharp left,
     * 0111 large left, 1011 small left; mirrored patterns turn right. */
    assert_turn_state(0x1U, -1, config.sharpInnerDuty, config.sharpOuterDuty);
    assert_turn_state(0x3U, -1, config.sharpInnerDuty, config.sharpOuterDuty);
    assert_turn_state(0x7U, -1, config.bigInnerDuty, config.bigOuterDuty);
    assert_turn_state(0xBU, -1, config.smallInnerDuty, config.smallOuterDuty);
    assert_turn_state(0x8U, 1, config.sharpInnerDuty, config.sharpOuterDuty);
    assert_turn_state(0xCU, 1, config.sharpInnerDuty, config.sharpOuterDuty);
    assert_turn_state(0xDU, 1, config.smallInnerDuty, config.smallOuterDuty);
    assert_turn_state(0xEU, 1, config.bigInnerDuty, config.bigOuterDuty);
    command = HTrackController_step(&controller, 0xFU, 120U, 4U);
    assert(command.drive == H_TRACK_DRIVE_HOLD_LAST);
    assert(command.direction == -1);

    /* A 0000 observation before the minimum lap distance is ordinary tape. */
    command = HTrackController_step(&controller, 0x0U, 499U, 4U);
    assert(controller.state == H_TRACK_STATE_FOLLOW);
    assert(command.drive != H_TRACK_DRIVE_STOP);

    /* Application can require an IMU lap before allowing the physical A mark. */
    HTrackController_setAMarkerArmed(&controller, 0U);
    command = HTrackController_step(&controller, 0x0U, 500U, 4U);
    assert(controller.state == H_TRACK_STATE_FOLLOW);
    assert(command.drive != H_TRACK_DRIVE_STOP);

    HTrackController_setAMarkerArmed(&controller, 1U);
    command = HTrackController_step(&controller, 0x0U, 500U, 4U);
    assert(controller.state == H_TRACK_STATE_CONFIRM_A_MARK);
    assert(command.drive == H_TRACK_DRIVE_HOLD_LAST);
    (void) HTrackController_step(&controller, 0x0U, 504U, 8U);
    command = HTrackController_step(&controller, 0x0U, 508U, 8U);
    assert(controller.state == H_TRACK_STATE_STOP_APPROACH);
    assert(controller.markerDetectedCounts == 508U);
    assert(controller.stopTargetCounts == 588U);
    assert(command.drive == H_TRACK_DRIVE_STOP_APPROACH);
    assert(command.innerDuty == config.approachDuty);

    command = HTrackController_step(&controller, 0x9U, 570U, 4U);
    assert(command.drive == H_TRACK_DRIVE_STOP_APPROACH);
    assert(command.innerDuty == config.finishDuty);
    command = HTrackController_step(&controller, 0x9U, 588U, 4U);
    assert(controller.state == H_TRACK_STATE_FINISHED);
    assert(command.drive == H_TRACK_DRIVE_STOP);

    puts("h_track_controller_test: PASS");
    return 0;
}
