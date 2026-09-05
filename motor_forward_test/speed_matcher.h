#ifndef SPEED_MATCHER_H_
#define SPEED_MATCHER_H_

#include <stdint.h>

/*
 * Keep the two encoder speeds matched while the line controller supplies
 * the steering demand.  A positive trim means the left wheel was faster:
 * reduce left PWM and raise right PWM by that amount.
 */
typedef struct {
    uint8_t deadbandCounts;
    uint8_t proportionalDivisorCounts;
    uint8_t integralDivisorCounts;
    uint8_t maxDutyTrim;
    int32_t integralLimitCounts;
} SpeedMatcherConfig;

typedef struct {
    SpeedMatcherConfig config;
    int32_t integralError;
    int32_t latestSpeedError;
    int8_t dutyTrim;
} SpeedMatcher;

void SpeedMatcher_init(SpeedMatcher *matcher, const SpeedMatcherConfig *config);

/* Update from one equal-duration left/right encoder-count window. */
int8_t SpeedMatcher_update(SpeedMatcher *matcher, int32_t leftDelta,
                           int32_t rightDelta);

/* Mix the speed trim into a line-controller PWM command and clamp it. */
void SpeedMatcher_apply(const SpeedMatcher *matcher, uint8_t requestedLeftDuty,
                        uint8_t requestedRightDuty, uint8_t minimumDuty,
                        uint8_t maximumDuty, uint8_t *leftDuty,
                        uint8_t *rightDuty);

#endif /* SPEED_MATCHER_H_ */
