#include <assert.h>
#include <string.h>

#include "task4_baseline_profile.h"

int main(void)
{
    Task4BaselineProfile profile;
    Task4BaselineCommand command;

    Task4Baseline_init(&profile, NULL);
    assert(profile.phase == TASK4_BASELINE_DISARMED);
    assert(!Task4Baseline_start(&profile, 100U));
    command = Task4Baseline_step(&profile, 100U);
    assert(!command.driveEnabled && command.dutyPercent == 0U);

    Task4Baseline_arm(&profile);
    assert(profile.phase == TASK4_BASELINE_READY);
    assert(Task4Baseline_start(&profile, 1000U));
    command = Task4Baseline_step(&profile, 1000U);
    assert(command.phase == TASK4_BASELINE_RAMP_UP);
    assert(command.dutyPercent == 0U);
    command = Task4Baseline_step(&profile, 1750U);
    assert(command.dutyPercent == 11U);
    command = Task4Baseline_step(&profile, 2500U);
    assert(command.phase == TASK4_BASELINE_CRUISE);
    assert(command.dutyPercent == 22U);
    command = Task4Baseline_step(&profile, 5000U);
    assert(command.phase == TASK4_BASELINE_RAMP_DOWN);
    assert(command.dutyPercent == 15U);
    command = Task4Baseline_step(&profile, 5999U);
    assert(command.phase == TASK4_BASELINE_RAMP_DOWN);
    assert(command.dutyPercent == 1U);
    command = Task4Baseline_step(&profile, 6000U);
    assert(command.phase == TASK4_BASELINE_STOPPED);
    assert(command.justStopped && !command.driveEnabled);
    assert(strcmp(Task4Baseline_phaseName(command.phase), "STOPPED") == 0);

    Task4Baseline_arm(&profile);
    assert(Task4Baseline_start(&profile, UINT32_MAX - 100U));
    command = Task4Baseline_step(&profile, 49U);
    assert(command.phase == TASK4_BASELINE_RAMP_UP);
    assert(command.dutyPercent == 2U);
    Task4Baseline_stop(&profile);
    command = Task4Baseline_step(&profile, 50U);
    assert(command.phase == TASK4_BASELINE_STOPPED);
    assert(command.dutyPercent == 0U && !command.driveEnabled);
    return 0;
}
