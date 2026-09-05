#include <assert.h>

#include "joint_line_balance_control.h"

int main(void)
{
    int16_t error;
    int32_t left, right;
    JointMotionProfile motion;
    JointMotionConfig motionConfig = {1000, 500, 1000};
    JointLinePd line;
    JointLinePdConfig lineConfig = {2, 0, 64, 500, 50, 20};
    JointWheelPi wheel;
    JointWheelPiConfig wheelConfig = {100, 20, 0, 10, 8, 60, 1U, 1000000};
    JointWheelOpenLoop openLoop;
    JointWheelOpenLoopConfig openLoopConfig = {500, 22U, 1U};
    JointAccelerationEstimator acceleration;
    JointBeamFeedforward beam;
    JointBeamFeedforwardConfig beamConfig = {100, 1, -2221, 1777, 50, 200, 300};
    JointBallPdConfig ballConfig = {20, 5, 10, 400, false};

    assert(JointLine_blackMaskToError(0x06U, &error) && error == 0);
    assert(JointLine_blackMaskToError(0x01U, &error) && error == -300);
    assert(!JointLine_blackMaskToError(0U, &error));
    JointLine_init(&line, &lineConfig);
    assert(JointLine_step(&line, true, -300, 5U));
    assert(line.correction < 0);
    assert(JointLine_step(&line, false, 0, 5U));
    assert(!JointLine_step(&line, false, 0, 20U));

    Joint_mixWheelTargets(1000, 200, 1100, &left, &right);
    assert(left == 733 && right == 1100);
    Joint_mixWheelTargets(100, 300, 1000, &left, &right);
    assert(left == 0 && right == 400);

    JointMotion_init(&motion, &motionConfig);
    JointMotion_setRunning(&motion, true);
    JointMotion_step(&motion, 20U);
    assert(motion.accelerationX100 == 20 && motion.speedX100 == 0);
    for (int i = 0; i < 200; ++i) JointMotion_step(&motion, 20U);
    assert(motion.speedX100 == 1000);
    JointMotion_setRunning(&motion, false);
    for (int i = 0; i < 300; ++i) JointMotion_step(&motion, 20U);
    assert(motion.speedX100 == 0 && motion.accelerationX100 == 0);

    JointWheelPi_init(&wheel, &wheelConfig);
    assert(JointWheelPi_step(&wheel, 500, 0, 20U) == 1U);
    assert(JointWheelPi_step(&wheel, 500, 0, 20U) == 2U);
    assert(JointWheelPi_step(&wheel, 0, 0, 20U) == 0U && wheel.integral == 0);

    JointWheelOpenLoop_init(&openLoop, &openLoopConfig);
    assert(JointWheelOpenLoop_step(&openLoop, 500) == 1U);
    for (int i = 0; i < 30; ++i) (void) JointWheelOpenLoop_step(&openLoop, 500);
    assert(openLoop.duty == 22U);
    assert(JointWheelOpenLoop_step(&openLoop, 0) == 21U);

    JointAcceleration_init(&acceleration);
    assert(JointAcceleration_step(&acceleration, 0, 20U) == 0);
    assert(JointAcceleration_step(&acceleration, 400, 20U) > 0);
    JointBeamFeedforward_init(&beam, &beamConfig);
    assert(JointBeamFeedforward_step(&beam, 10000, 0, 0) > 0);
    assert(JointBeamFeedforward_step(&beam, 10000, 300, 0) < 50);
    assert(JointBallPd_step(&ballConfig, true, 100, 0) == 0);
    ballConfig.enabled = true;
    assert(JointBallPd_step(&ballConfig, true, 100, 0) == 20);
    assert(JointBallPd_step(&ballConfig, false, 100, 0) == 0);
    ballConfig = (JointBallPdConfig){-130, -1200, 10, 400, true};
    assert(JointBallPd_step(&ballConfig, true, 200, 0) == -260);
    return 0;
}
