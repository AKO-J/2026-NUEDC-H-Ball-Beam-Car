#include <assert.h>

#include "speed_matcher.h"

static SpeedMatcherConfig default_config(void)
{
    const SpeedMatcherConfig config = {
        .deadbandCounts = 2U,
        .proportionalDivisorCounts = 8U,
        .integralDivisorCounts = 32U,
        .maxDutyTrim = 8U,
        .integralLimitCounts = 256,
    };
    return config;
}

int main(void)
{
    SpeedMatcher matcher;
    uint8_t leftDuty;
    uint8_t rightDuty;
    const SpeedMatcherConfig config = default_config();

    SpeedMatcher_init(&matcher, &config);

    /* Left has more pulses: it is faster and must receive less PWM. */
    assert(SpeedMatcher_update(&matcher, 120, 96) > 0);
    SpeedMatcher_apply(&matcher, 30U, 30U, 20U, 48U,
                       &leftDuty, &rightDuty);
    assert(leftDuty < rightDuty);

    SpeedMatcher_init(&matcher, &config);
    /* Right has more pulses: give the left wheel more PWM. */
    assert(SpeedMatcher_update(&matcher, 94, 120) < 0);
    SpeedMatcher_apply(&matcher, 30U, 30U, 20U, 48U,
                       &leftDuty, &rightDuty);
    assert(leftDuty > rightDuty);

    SpeedMatcher_init(&matcher, &config);
    /* Tiny encoder differences stay inside the configured deadband. */
    assert(SpeedMatcher_update(&matcher, 100, 101) == 0);

    return 0;
}
