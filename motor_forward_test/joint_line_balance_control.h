#ifndef JOINT_LINE_BALANCE_CONTROL_H
#define JOINT_LINE_BALANCE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* All speed and acceleration values use x100 engineering units. */
typedef struct {
    int16_t kpSpeedPerError;
    int16_t kiSpeedPerErrorSecond;
    int16_t kdSpeedPerErrorPerSecond;
    int16_t feedforwardDutyPerSpeed;
    int16_t minimumMovingDuty;
    int16_t maximumDuty;
    uint8_t maximumDutyStepPerUpdate;
    int32_t integralLimit;
} JointWheelPiConfig;

typedef struct {
    JointWheelPiConfig config;
    int32_t integral;
    int32_t previousError;
    int32_t error;
    uint8_t duty;
} JointWheelPi;

/* Open-loop wheel command used by the first joint bring-up run.  It maps the
 * already-smoothed speed reference to PWM; encoder feedback remains available
 * for logging, acceleration feed-forward and stall detection, but does not
 * alter motor PWM. */
typedef struct {
    int32_t speedAtMaximumDutyX100;
    uint8_t maximumDuty;
    uint8_t maximumDutyStepPerUpdate;
} JointWheelOpenLoopConfig;

typedef struct {
    JointWheelOpenLoopConfig config;
    uint8_t duty;
} JointWheelOpenLoop;

typedef struct {
    int16_t kpCorrectionPerError;
    int16_t kdCorrectionPerRate;
    int16_t derivativeAlpha256;
    int16_t maximumCorrection;
    int16_t maximumCorrectionStep;
    uint16_t lostHoldMs;
} JointLinePdConfig;

typedef struct {
    JointLinePdConfig config;
    int16_t previousError;
    int16_t filteredRate;
    int16_t correction;
    uint16_t lostMs;
    bool initialized;
} JointLinePd;

typedef struct {
    int32_t cruiseSpeedX100;
    int32_t accelerationLimitX100;
    int32_t jerkLimitX100;
} JointMotionConfig;

typedef struct {
    JointMotionConfig config;
    int32_t speedX100;
    int32_t accelerationX100;
    int32_t targetSpeedX100;
} JointMotionProfile;

typedef struct {
    int32_t filteredSpeedX100;
    int32_t previousSpeedX100;
    int32_t filteredAccelerationX100;
    bool initialized;
} JointAccelerationEstimator;

typedef struct {
    int16_t kffMilli;
    int8_t sign;
    int16_t minimumAngleMdeg;
    int16_t maximumAngleMdeg;
    int16_t maximumAngleStepMdeg;
    int16_t maximumSteeringCorrectionX100;
    int16_t maximumWheelDifferenceX100;
} JointBeamFeedforwardConfig;

typedef struct {
    JointBeamFeedforwardConfig config;
    int16_t angleMdeg;
} JointBeamFeedforward;

typedef struct {
    int16_t kpMdegPerCmX100;
    int16_t kdMdegPerCmPerSX100;
    int16_t deadbandCmX100;
    int16_t maximumCorrectionMdeg;
    bool enabled;
} JointBallPdConfig;

bool JointLine_blackMaskToError(uint8_t leftToRightBlackMask,
                                int16_t *error);
void JointLine_init(JointLinePd *controller, const JointLinePdConfig *config);
bool JointLine_step(JointLinePd *controller, bool lineVisible,
                    int16_t error, uint16_t dtMs);

void JointWheelPi_init(JointWheelPi *controller,
                       const JointWheelPiConfig *config);
uint8_t JointWheelPi_step(JointWheelPi *controller, int32_t targetSpeedX100,
                          int32_t measuredSpeedX100, uint16_t dtMs);

void JointWheelOpenLoop_init(JointWheelOpenLoop *controller,
                             const JointWheelOpenLoopConfig *config);
uint8_t JointWheelOpenLoop_step(JointWheelOpenLoop *controller,
                                int32_t targetSpeedX100);

void JointMotion_init(JointMotionProfile *profile,
                      const JointMotionConfig *config);
void JointMotion_setRunning(JointMotionProfile *profile, bool running);
void JointMotion_step(JointMotionProfile *profile, uint16_t dtMs);

void JointAcceleration_init(JointAccelerationEstimator *estimator);
int32_t JointAcceleration_step(JointAccelerationEstimator *estimator,
                               int32_t vehicleSpeedX100, uint16_t dtMs);

void JointBeamFeedforward_init(JointBeamFeedforward *feedforward,
                               const JointBeamFeedforwardConfig *config);
int16_t JointBeamFeedforward_step(JointBeamFeedforward *feedforward,
                                  int32_t accelerationX100,
                                  int16_t steeringCorrectionX100,
                                  int32_t wheelDifferenceX100);
int16_t JointBallPd_step(const JointBallPdConfig *config,
                         bool visionFresh, int32_t errorCmX100,
                         int32_t velocityCmPerSX100);

/* Preserves base speed/correction ratio while preventing reverse targets. */
void Joint_mixWheelTargets(int32_t baseSpeedX100, int32_t correctionX100,
                           int32_t maximumSpeedX100, int32_t *leftTargetX100,
                           int32_t *rightTargetX100);

#endif
