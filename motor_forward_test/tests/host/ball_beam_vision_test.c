#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ball_beam_controller.h"
#include "beam_calibration.h"
#include "beam_level_reference.h"
#include "vision_ball_protocol.h"

static void feedFrame(
    VisionBallParser *parser,
    const char *payload,
    VisionBallMeasurement *measurement)
{
    char frame[80];
    size_t length;
    size_t i;
    int count = snprintf(frame, sizeof(frame), "%s\r\n", payload);

    assert(count > 0);
    length = (size_t) count;
    for (i = 0U; i < length; ++i) {
        const bool complete = VisionBallParser_feed(
            parser, (unsigned char) frame[i], measurement);
        if (i + 1U == length - 1U) {
            assert(complete);
        } else {
            assert(!complete);
        }
    }
}

static void testCalibration(void)
{
    BeamCalibrationInterpolation result;

    assert(BeamCalibration_angleToPos(0, &result));
    assert(result.targetPos == 0);
    assert(BeamCalibration_angleToPos(1000, &result));
    assert(result.angleLowMdeg == 444);
    assert(result.angleHighMdeg == 1110);
    assert(result.posLow == 44);
    assert(result.posHigh == 88);
    assert(result.targetPos == 81);
    assert(BeamCalibration_angleToPos(-1000, &result));
    assert(result.angleLowMdeg == -1333);
    assert(result.angleHighMdeg == -444);
    assert(result.targetPos == -72);
    assert(BeamCalibration_angleToPos(-2221, &result));
    assert(result.targetPos == -132);
    assert(BeamCalibration_angleToPos(1777, &result));
    assert(result.targetPos == 132);
    assert(!BeamCalibration_angleToPos(-2222, &result));
    assert(!BeamCalibration_angleToPos(1778, &result));
}

static void testLevelReference(void)
{
    int32_t target;

    /* P_level=+12: all calibrated POS values remain relative to it. */
    assert(BeamLevelReference_makeTarget(12, 0, &target));
    assert(target == 12);
    assert(BeamLevelReference_makeTarget(12, 40, &target));
    assert(target == 52);
    assert(BeamLevelReference_makeTarget(12, -77, &target));
    assert(target == -65);
    assert(BeamLevelReference_makeTarget(12, -40, &target));
    assert(target == -28);
    assert(!BeamLevelReference_makeTarget(12, -133, &target));
    assert(!BeamLevelReference_makeTarget(12, 133, &target));
}

static void testProtocol(void)
{
    VisionBallParser parser;
    VisionBallMeasurement measurement;
    const char bad[] = "B,9,330,-18,872,2\r\n";
    size_t i;

    VisionBallParser_init(&parser);
    feedFrame(&parser, "B,123,45678,-18,872,0", &measurement);
    assert(measurement.frame == 123U);
    assert(measurement.k230Ms == 45678U);
    assert(measurement.xOffsetPx == -18);
    assert(measurement.confidenceMilli == 872U);
    assert(measurement.lost == 0U);
    assert(parser.acceptedFrames == 1U);

    for (i = 0U; i < sizeof(bad) - 1U; ++i) {
        assert(!VisionBallParser_feed(
            &parser, (unsigned char) bad[i], &measurement));
    }
    assert(parser.rejectedFrames == 1U);
}

static void testController(void)
{
    BallBeamController controller;
    BallBeamCommand command;

    BallBeamController_init(&controller, 160);
    assert(BallBeamController_update(&controller, -20, 100U, 1, &command));
    assert(command.errorPx == 20);
    assert(command.requestedAngleMdeg == 220);
    assert(command.calibration.targetPos == 22);

    assert(BallBeamController_update(&controller, -160, 120U, 1, &command));
    assert(command.requestedAngleMdeg == 1777);
    assert(command.calibration.targetPos == 132);
    assert(command.velocityPxPerS < 0);
    assert(command.predictedXOffsetPx < -160);

    BallBeamController_init(&controller, 160);
    assert(BallBeamController_update(&controller, 160, 100U, 1, &command));
    assert(command.requestedAngleMdeg == -1760);
    assert(command.calibration.targetPos >= -132);
    assert(command.calibration.targetPos <= 132);
}

int main(void)
{
    testCalibration();
    testLevelReference();
    testProtocol();
    testController();
    puts("ball_beam_vision_test: PASS");
    return 0;
}
