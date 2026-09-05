#include "joint_line_balance_control.h"

#include <limits.h>
#include <stddef.h>

static int32_t clamp32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int32_t approach(int32_t value, int32_t target, int32_t maximumStep)
{
    if (value < target) {
        return value + ((target - value > maximumStep) ? maximumStep : target - value);
    }
    if (value > target) {
        return value - ((value - target > maximumStep) ? maximumStep : value - target);
    }
    return value;
}

bool JointLine_blackMaskToError(uint8_t mask, int16_t *error)
{
    static const int8_t weights[4] = {-3, -1, 1, 3};
    int16_t sum = 0;
    int16_t count = 0;
    uint8_t index;
    if (error == NULL) return false;
    for (index = 0U; index < 4U; ++index) {
        if ((mask & ((uint8_t) 1U << index)) != 0U) {
            sum += weights[index];
            ++count;
        }
    }
    if (count == 0) return false;
    *error = (int16_t) ((sum * 100) / count);
    return true;
}

void JointLine_init(JointLinePd *controller, const JointLinePdConfig *config)
{
    if ((controller == NULL) || (config == NULL)) return;
    controller->config = *config;
    controller->previousError = 0;
    controller->filteredRate = 0;
    controller->correction = 0;
    controller->lostMs = 0U;
    controller->initialized = false;
}

bool JointLine_step(JointLinePd *controller, bool visible,
                    int16_t error, uint16_t dtMs)
{
    int32_t rawRate;
    int32_t requested;
    int32_t alpha;
    if ((controller == NULL) || (dtMs == 0U)) return false;
    if (!visible) {
        controller->lostMs = (controller->lostMs > 65535U - dtMs) ?
            65535U : (uint16_t) (controller->lostMs + dtMs);
        return controller->lostMs <= controller->config.lostHoldMs;
    }
    controller->lostMs = 0U;
    if (!controller->initialized) {
        controller->previousError = error;
        controller->initialized = true;
    }
    rawRate = ((int32_t) error - controller->previousError) * 1000 / dtMs;
    alpha = clamp32(controller->config.derivativeAlpha256, 0, 256);
    controller->filteredRate = (int16_t) clamp32(
        ((int32_t) controller->filteredRate * (256 - alpha) + rawRate * alpha) / 256,
        INT16_MIN, INT16_MAX);
    requested = ((int32_t) controller->config.kpCorrectionPerError * error) / 100 +
        ((int32_t) controller->config.kdCorrectionPerRate * controller->filteredRate) / 1000;
    requested = clamp32(requested, -controller->config.maximumCorrection,
                                   controller->config.maximumCorrection);
    controller->correction = (int16_t) approach(controller->correction,
        requested, controller->config.maximumCorrectionStep);
    controller->previousError = error;
    return true;
}

void JointWheelPi_init(JointWheelPi *controller,
                       const JointWheelPiConfig *config)
{
    if ((controller == NULL) || (config == NULL)) return;
    controller->config = *config;
    controller->integral = 0;
    controller->previousError = 0;
    controller->error = 0;
    controller->duty = 0U;
}

uint8_t JointWheelPi_step(JointWheelPi *controller, int32_t target,
                          int32_t measured, uint16_t dtMs)
{
    int32_t candidateIntegral;
    int32_t derivative;
    int32_t output;
    if ((controller == NULL) || (dtMs == 0U) || (target <= 0)) {
        if (controller != NULL) {
            controller->integral = 0; controller->error = 0;
            controller->previousError = 0; controller->duty = 0U;
        }
        return 0U;
    }
    controller->error = target - measured;
    candidateIntegral = clamp32(controller->integral + controller->error * dtMs,
        -controller->config.integralLimit, controller->config.integralLimit);
    derivative = (controller->error - controller->previousError) * 1000 / dtMs;
    output = controller->config.minimumMovingDuty +
        ((int32_t) controller->config.feedforwardDutyPerSpeed * target) / 10000 +
        ((int32_t) controller->config.kpSpeedPerError * controller->error) / 1000 +
        ((int32_t) controller->config.kiSpeedPerErrorSecond * candidateIntegral) / 1000000 +
        ((int32_t) controller->config.kdSpeedPerErrorPerSecond * derivative) / 100000;
    /* Conditional integration: do not wind farther into a saturated output. */
    if (!((output >= controller->config.maximumDuty && controller->error > 0) ||
          (output <= 0 && controller->error < 0))) {
        controller->integral = candidateIntegral;
    }
    output = clamp32(output, 0, controller->config.maximumDuty);
    output = approach(controller->duty, output,
        (controller->config.maximumDutyStepPerUpdate == 0U) ? 1 :
        controller->config.maximumDutyStepPerUpdate);
    controller->previousError = controller->error;
    controller->duty = (uint8_t) output;
    return controller->duty;
}

void JointWheelOpenLoop_init(JointWheelOpenLoop *controller,
                             const JointWheelOpenLoopConfig *config)
{
    if ((controller == NULL) || (config == NULL)) return;
    controller->config = *config;
    controller->duty = 0U;
}

uint8_t JointWheelOpenLoop_step(JointWheelOpenLoop *controller,
                                int32_t targetSpeedX100)
{
    int32_t requested;
    int32_t maximumStep;
    if (controller == NULL) return 0U;
    if ((targetSpeedX100 <= 0) || (controller->config.speedAtMaximumDutyX100 <= 0))
        requested = 0;
    else
        requested = targetSpeedX100 * controller->config.maximumDuty /
                    controller->config.speedAtMaximumDutyX100;
    requested = clamp32(requested, 0, controller->config.maximumDuty);
    maximumStep = (controller->config.maximumDutyStepPerUpdate == 0U) ? 1 :
                  controller->config.maximumDutyStepPerUpdate;
    controller->duty = (uint8_t) approach(controller->duty, requested, maximumStep);
    return controller->duty;
}

void JointMotion_init(JointMotionProfile *profile,
                      const JointMotionConfig *config)
{
    if ((profile == NULL) || (config == NULL)) return;
    profile->config = *config;
    profile->speedX100 = 0;
    profile->accelerationX100 = 0;
    profile->targetSpeedX100 = 0;
}

void JointMotion_setRunning(JointMotionProfile *profile, bool running)
{
    if (profile != NULL) profile->targetSpeedX100 = running ? profile->config.cruiseSpeedX100 : 0;
}

void JointMotion_step(JointMotionProfile *profile, uint16_t dtMs)
{
    int32_t desiredAcceleration;
    int32_t accelStep;
    int32_t nextSpeed;
    if ((profile == NULL) || (dtMs == 0U)) return;
    desiredAcceleration = (profile->speedX100 < profile->targetSpeedX100) ?
        profile->config.accelerationLimitX100 : -profile->config.accelerationLimitX100;
    if (profile->speedX100 == profile->targetSpeedX100) desiredAcceleration = 0;
    accelStep = profile->config.jerkLimitX100 * dtMs / 1000;
    if (accelStep < 1) accelStep = 1;
    profile->accelerationX100 = approach(profile->accelerationX100,
                                         desiredAcceleration, accelStep);
    nextSpeed = profile->speedX100 + profile->accelerationX100 * dtMs / 1000;
    if (((profile->targetSpeedX100 >= profile->speedX100) &&
         (nextSpeed >= profile->targetSpeedX100)) ||
        ((profile->targetSpeedX100 <= profile->speedX100) &&
         (nextSpeed <= profile->targetSpeedX100))) {
        nextSpeed = profile->targetSpeedX100;
        if (nextSpeed == 0) profile->accelerationX100 = 0;
    }
    profile->speedX100 = (nextSpeed < 0) ? 0 : nextSpeed;
}

void JointAcceleration_init(JointAccelerationEstimator *estimator)
{
    if (estimator == NULL) return;
    estimator->filteredSpeedX100 = 0;
    estimator->previousSpeedX100 = 0;
    estimator->filteredAccelerationX100 = 0;
    estimator->initialized = false;
}

int32_t JointAcceleration_step(JointAccelerationEstimator *estimator,
                               int32_t speed, uint16_t dtMs)
{
    int32_t rawAcceleration;
    if ((estimator == NULL) || (dtMs == 0U)) return 0;
    estimator->filteredSpeedX100 = (estimator->filteredSpeedX100 * 3 + speed) / 4;
    if (!estimator->initialized) {
        estimator->previousSpeedX100 = estimator->filteredSpeedX100;
        estimator->initialized = true;
        return 0;
    }
    rawAcceleration = (estimator->filteredSpeedX100 - estimator->previousSpeedX100) * 1000 / dtMs;
    rawAcceleration = clamp32(rawAcceleration, -30000, 30000);
    estimator->filteredAccelerationX100 =
        (estimator->filteredAccelerationX100 * 3 + rawAcceleration) / 4;
    estimator->previousSpeedX100 = estimator->filteredSpeedX100;
    return estimator->filteredAccelerationX100;
}

void JointBeamFeedforward_init(JointBeamFeedforward *feedforward,
                               const JointBeamFeedforwardConfig *config)
{
    if ((feedforward == NULL) || (config == NULL)) return;
    feedforward->config = *config;
    feedforward->angleMdeg = 0;
}

int16_t JointBeamFeedforward_step(JointBeamFeedforward *ff,
                                  int32_t accelerationX100,
                                  int16_t steeringCorrectionX100,
                                  int32_t wheelDifferenceX100)
{
    int32_t requested = 0;
    if (ff == NULL) return 0;
    if ((steeringCorrectionX100 <= ff->config.maximumSteeringCorrectionX100) &&
        (steeringCorrectionX100 >= -ff->config.maximumSteeringCorrectionX100) &&
        (wheelDifferenceX100 <= ff->config.maximumWheelDifferenceX100) &&
        (wheelDifferenceX100 >= -ff->config.maximumWheelDifferenceX100)) {
        /* atan(x) ~= x: max configured acceleration is <0.31 g; error is
         * bounded further by the measured beam calibration limits. */
        requested = accelerationX100 * 57296L / 98067L;
        requested = requested * ff->config.kffMilli / 1000;
        requested *= ff->config.sign;
    }
    requested = clamp32(requested, ff->config.minimumAngleMdeg,
                                    ff->config.maximumAngleMdeg);
    ff->angleMdeg = (int16_t) approach(ff->angleMdeg, requested,
                                      ff->config.maximumAngleStepMdeg);
    return ff->angleMdeg;
}

int16_t JointBallPd_step(const JointBallPdConfig *config,
                         bool visionFresh, int32_t error,
                         int32_t velocity)
{
    int32_t output;
    if ((config == NULL) || !config->enabled || !visionFresh) return 0;
    if (error >= -config->deadbandCmX100 && error <= config->deadbandCmX100)
        error = 0;
    output = ((int32_t) config->kpMdegPerCmX100 * error) / 100 +
             ((int32_t) config->kdMdegPerCmPerSX100 * velocity) / 100;
    return (int16_t) clamp32(output, -config->maximumCorrectionMdeg,
                                     config->maximumCorrectionMdeg);
}

void Joint_mixWheelTargets(int32_t base, int32_t correction, int32_t maximum,
                           int32_t *left, int32_t *right)
{
    int32_t l;
    int32_t r;
    int32_t peak;
    if ((left == NULL) || (right == NULL)) return;
    l = base - correction;
    r = base + correction;
    if (l < 0) l = 0;
    if (r < 0) r = 0;
    peak = (l > r) ? l : r;
    if ((peak > maximum) && (peak > 0)) {
        l = l * maximum / peak;
        r = r * maximum / peak;
    }
    *left = l;
    *right = r;
}
