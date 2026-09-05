#ifndef TASK4_BASELINE_PROFILE_H
#define TASK4_BASELINE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TASK4_BASELINE_DISARMED = 0,
    TASK4_BASELINE_READY,
    TASK4_BASELINE_RAMP_UP,
    TASK4_BASELINE_CRUISE,
    TASK4_BASELINE_RAMP_DOWN,
    TASK4_BASELINE_STOPPED,
} Task4BaselinePhase;

typedef struct {
    uint16_t rampUpMs;
    uint16_t cruiseMs;
    uint16_t rampDownMs;
    uint8_t cruiseDutyPercent;
} Task4BaselineConfig;

typedef struct {
    Task4BaselineConfig config;
    Task4BaselinePhase phase;
    uint32_t startedMs;
} Task4BaselineProfile;

typedef struct {
    Task4BaselinePhase phase;
    uint8_t dutyPercent;
    bool driveEnabled;
    bool justStopped;
} Task4BaselineCommand;

/* Conservative first wheel-off-ground baseline; change only after its log is
 * reviewed.  This profile does not contain ball-controller/PID parameters. */
extern const Task4BaselineConfig k_task4_baseline_default_config;

void Task4Baseline_init(Task4BaselineProfile *profile,
                        const Task4BaselineConfig *config);
void Task4Baseline_arm(Task4BaselineProfile *profile);
bool Task4Baseline_start(Task4BaselineProfile *profile, uint32_t nowMs);
void Task4Baseline_stop(Task4BaselineProfile *profile);
Task4BaselineCommand Task4Baseline_step(Task4BaselineProfile *profile,
                                        uint32_t nowMs);
const char *Task4Baseline_phaseName(Task4BaselinePhase phase);

#endif
