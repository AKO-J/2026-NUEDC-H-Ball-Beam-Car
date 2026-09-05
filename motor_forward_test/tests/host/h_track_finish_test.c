#include <assert.h>
#include <stdio.h>

#include "h_track_finish.h"

int main(void)
{
    HTrackFinishDetector detector;
    const HTrackFinishConfig config = {
        .slowStartDistance = 58000U,
        .stopDistance = 61416U,
    };

    HTrackFinishDetector_init(&detector, &config);

    assert(HTrackFinishDetector_step(&detector, 57999U) == 0U);
    assert(detector.armed == 0U);

    /* The final slow window is armed without stopping the car. */
    assert(HTrackFinishDetector_step(&detector, 58000U) == 0U);
    assert(detector.armed == 1U);

    assert(HTrackFinishDetector_step(&detector, 61415U) == 0U);
    assert(HTrackFinishDetector_step(&detector, 61416U) == 1U);
    assert(detector.reached == 1U);
    assert(HTrackFinishDetector_step(&detector, 0U) == 1U);

    puts("h_track_finish_test: PASS");
    return 0;
}
