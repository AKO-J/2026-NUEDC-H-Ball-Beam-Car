#include "speed_matcher.h"

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint8_t clamp_duty(int32_t duty, uint8_t minimum, uint8_t maximum)
{
    return (uint8_t) clamp_i32(duty, (int32_t) minimum, (int32_t) maximum);
}

void SpeedMatcher_init(SpeedMatcher *matcher, const SpeedMatcherConfig *config)
{
    matcher->config = *config;
    if (matcher->config.proportionalDivisorCounts == 0U) {
        matcher->config.proportionalDivisorCounts = 1U;
    }
    if (matcher->config.integralDivisorCounts == 0U) {
        matcher->config.integralDivisorCounts = 1U;
    }
    if (matcher->config.integralLimitCounts < 1) {
        matcher->config.integralLimitCounts = 1;
    }

    matcher->integralError = 0;
    matcher->latestSpeedError = 0;
    matcher->dutyTrim = 0;
}

int8_t SpeedMatcher_update(SpeedMatcher *matcher, int32_t leftDelta,
                           int32_t rightDelta)
{
    int32_t speedError = leftDelta - rightDelta;
    int32_t correction;

    if ((speedError >= -(int32_t) matcher->config.deadbandCounts) &&
        (speedError <= (int32_t) matcher->config.deadbandCounts)) {
        speedError = 0;
    }

    /* Leaking the integral prevents an old turn from biasing a new straight. */
    matcher->integralError = clamp_i32((matcher->integralError * 7) / 8 +
                                       speedError,
                                       -matcher->config.integralLimitCounts,
                                        matcher->config.integralLimitCounts);
    correction = (speedError /
                  (int32_t) matcher->config.proportionalDivisorCounts) +
                 (matcher->integralError /
                  (int32_t) matcher->config.integralDivisorCounts);
    correction = clamp_i32(correction, -(int32_t) matcher->config.maxDutyTrim,
                                       (int32_t) matcher->config.maxDutyTrim);

    matcher->latestSpeedError = speedError;
    matcher->dutyTrim = (int8_t) correction;
    return matcher->dutyTrim;
}

void SpeedMatcher_apply(const SpeedMatcher *matcher, uint8_t requestedLeftDuty,
                        uint8_t requestedRightDuty, uint8_t minimumDuty,
                        uint8_t maximumDuty, uint8_t *leftDuty,
                        uint8_t *rightDuty)
{
    const int32_t trim = (int32_t) matcher->dutyTrim;

    *leftDuty = clamp_duty((int32_t) requestedLeftDuty - trim,
                           minimumDuty, maximumDuty);
    *rightDuty = clamp_duty((int32_t) requestedRightDuty + trim,
                            minimumDuty, maximumDuty);
}
