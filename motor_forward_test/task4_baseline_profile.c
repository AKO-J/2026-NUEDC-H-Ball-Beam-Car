#include "task4_baseline_profile.h"

#include <stddef.h>

const Task4BaselineConfig k_task4_baseline_default_config = {
    .rampUpMs = 1500U,
    .cruiseMs = 2000U,
    .rampDownMs = 1500U,
    .cruiseDutyPercent = 22U,
};

static uint8_t ramp_duty(uint32_t elapsedMs, uint16_t durationMs,
                         uint8_t maximum, bool rising)
{
    uint32_t scaled;

    if (durationMs == 0U) {
        return rising ? maximum : 0U;
    }
    if (elapsedMs >= durationMs) {
        return rising ? maximum : 0U;
    }
    scaled = ((uint32_t) maximum * elapsedMs) / durationMs;
    return rising ? (uint8_t) scaled : (uint8_t) (maximum - scaled);
}

void Task4Baseline_init(Task4BaselineProfile *profile,
                        const Task4BaselineConfig *config)
{
    if (profile == NULL) {
        return;
    }
    profile->config = (config == NULL) ?
        k_task4_baseline_default_config : *config;
    if (profile->config.cruiseDutyPercent > 100U) {
        profile->config.cruiseDutyPercent = 100U;
    }
    profile->phase = TASK4_BASELINE_DISARMED;
    profile->startedMs = 0U;
}

void Task4Baseline_arm(Task4BaselineProfile *profile)
{
    if ((profile != NULL) &&
        ((profile->phase == TASK4_BASELINE_DISARMED) ||
         (profile->phase == TASK4_BASELINE_STOPPED))) {
        profile->phase = TASK4_BASELINE_READY;
    }
}

bool Task4Baseline_start(Task4BaselineProfile *profile, uint32_t nowMs)
{
    if ((profile == NULL) || (profile->phase != TASK4_BASELINE_READY)) {
        return false;
    }
    profile->startedMs = nowMs;
    profile->phase = TASK4_BASELINE_RAMP_UP;
    return true;
}

void Task4Baseline_stop(Task4BaselineProfile *profile)
{
    if (profile != NULL) {
        profile->phase = TASK4_BASELINE_STOPPED;
    }
}

Task4BaselineCommand Task4Baseline_step(Task4BaselineProfile *profile,
                                        uint32_t nowMs)
{
    Task4BaselineCommand command = {TASK4_BASELINE_DISARMED, 0U, false, false};
    uint32_t elapsed;
    uint32_t boundary;

    if (profile == NULL) {
        return command;
    }
    command.phase = profile->phase;
    if ((profile->phase != TASK4_BASELINE_RAMP_UP) &&
        (profile->phase != TASK4_BASELINE_CRUISE) &&
        (profile->phase != TASK4_BASELINE_RAMP_DOWN)) {
        return command;
    }

    elapsed = nowMs - profile->startedMs;
    boundary = profile->config.rampUpMs;
    if (elapsed < boundary) {
        profile->phase = TASK4_BASELINE_RAMP_UP;
        command.dutyPercent = ramp_duty(elapsed, profile->config.rampUpMs,
                                        profile->config.cruiseDutyPercent, true);
    } else {
        boundary += profile->config.cruiseMs;
        if (elapsed < boundary) {
            profile->phase = TASK4_BASELINE_CRUISE;
            command.dutyPercent = profile->config.cruiseDutyPercent;
        } else {
            boundary += profile->config.rampDownMs;
            if (elapsed < boundary) {
                profile->phase = TASK4_BASELINE_RAMP_DOWN;
                command.dutyPercent = ramp_duty(
                    elapsed - profile->config.rampUpMs - profile->config.cruiseMs,
                    profile->config.rampDownMs,
                    profile->config.cruiseDutyPercent, false);
            } else {
                profile->phase = TASK4_BASELINE_STOPPED;
                command.phase = profile->phase;
                command.justStopped = true;
                return command;
            }
        }
    }
    command.phase = profile->phase;
    command.driveEnabled = command.dutyPercent != 0U;
    return command;
}

const char *Task4Baseline_phaseName(Task4BaselinePhase phase)
{
    switch (phase) {
    case TASK4_BASELINE_DISARMED: return "DISARMED";
    case TASK4_BASELINE_READY: return "READY";
    case TASK4_BASELINE_RAMP_UP: return "RAMP_UP";
    case TASK4_BASELINE_CRUISE: return "CRUISE";
    case TASK4_BASELINE_RAMP_DOWN: return "RAMP_DOWN";
    case TASK4_BASELINE_STOPPED: return "STOPPED";
    default: return "INVALID";
    }
}
