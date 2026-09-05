/*
 * Motor-on IMU interference recorder.
 *
 * Secure the chassis and suspend both drive wheels before pressing S2.  The
 * body must not rotate: every phase heading delta is then unwanted gyro drift
 * caused by motor vibration, PWM noise, or supply disturbance.
 *
 * ICM:  VCC=3V3, GND=GND, SDA=PA28, SCL=PA31, CS=3V3, AD0=GND
 * OLED: VCC=3V3, GND=GND, SCL=PB19, SDA=PA15
 * S2:   PB7 -> GND (starts the sequence only after calibration is complete)
 */
#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "encoder.h"
#include "icm42688.h"
#include "imu_yaw.h"
#include "motor_driver.h"
#include "ssd1306_oled.h"

#define TEST_TICK_MS                        5U
#define TEST_PHASE_MS                    3000U
#define TEST_RESULT_HOLD_MS               5000U
#define TEST_IMU_START_RETRIES               3U
#define TEST_IMU_RETRY_GAP_MS             100U

#define TEST_BUTTON_PORT                  GPIOB
#define TEST_BUTTON_PIN                   DL_GPIO_PIN_7
#define TEST_BUTTON_IOMUX                 IOMUX_PINCM24
#define TEST_BUTTON_DEBOUNCE_MS            30U
#define TEST_SYSTICK_RELOAD          0x00FFFFFFU

#define TEST_PHASE_COUNT                    6U

typedef struct {
    const char *name;
    uint8_t leftDuty;
    uint8_t rightDuty;
} TestPhase;

typedef struct {
    int32_t headingStartMdeg;
    int32_t leftEncoderStart;
    int32_t rightEncoderStart;
    int64_t correctedSum;
    uint32_t correctedPeak;
    uint16_t validSamples;
} PhaseRecord;

/* CCS-watchable results.  Index 0..5 matches k_test_phases. */
volatile uint8_t g_motor_imu_phase;
volatile uint8_t g_motor_imu_left_duty;
volatile uint8_t g_motor_imu_right_duty;
volatile int32_t g_motor_imu_base_bias_raw;
volatile int32_t g_motor_imu_phase_delta_mdeg[TEST_PHASE_COUNT];
volatile int32_t g_motor_imu_average_raw[TEST_PHASE_COUNT];
volatile uint32_t g_motor_imu_peak_raw[TEST_PHASE_COUNT];
volatile int32_t g_motor_imu_left_encoder_delta[TEST_PHASE_COUNT];
volatile int32_t g_motor_imu_right_encoder_delta[TEST_PHASE_COUNT];
volatile uint16_t g_motor_imu_valid_samples[TEST_PHASE_COUNT];
volatile uint32_t g_motor_imu_failures;
volatile int16_t g_motor_imu_raw_z;
volatile int32_t g_motor_imu_corrected_raw;

static const TestPhase k_test_phases[TEST_PHASE_COUNT] = {
    {"STILL",  0U,  0U},
    {"FWD",   38U, 40U},
    {"LEFT",  38U,  0U},
    {"RIGHT",  0U, 40U},
    {"LTURN", 28U, 50U},
    {"RTURN", 50U, 28U},
};

static uint32_t magnitude_i32(int32_t value)
{
    return (value < 0) ? (uint32_t) (-(value + 1)) + 1U : (uint32_t) value;
}

static void wait_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_cycles(32000U);
    }
}

static void systick_init(void)
{
    SysTick->CTRL = 0U;
    SysTick->LOAD = TEST_SYSTICK_RELOAD;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static void button_init(void)
{
    DL_GPIO_enablePower(TEST_BUTTON_PORT);
    delay_cycles(16U);
    DL_GPIO_initDigitalInputFeatures(TEST_BUTTON_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

static uint8_t button_pressed(void)
{
    return (DL_GPIO_readPins(TEST_BUTTON_PORT, TEST_BUTTON_PIN) == 0U) ? 1U : 0U;
}

static void line_clear(char *line)
{
    uint8_t index;

    for (index = 0U; index <= OLED_TEXT_LINE_MAX_CHARS; index++) {
        line[index] = '\0';
    }
}

static uint8_t append_char(char *line, uint8_t index, char value)
{
    if (index < OLED_TEXT_LINE_MAX_CHARS) {
        line[index++] = value;
        line[index] = '\0';
    }
    return index;
}

static uint8_t append_text(char *line, uint8_t index, const char *text)
{
    while (*text != '\0') {
        index = append_char(line, index, *text++);
    }
    return index;
}

static uint8_t append_unsigned(char *line, uint8_t index, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(digits)));

    while (count != 0U) {
        index = append_char(line, index, digits[--count]);
    }
    return index;
}

static uint8_t append_signed(char *line, uint8_t index, int32_t value)
{
    index = append_char(line, index, (value < 0) ? '-' : '+');
    return append_unsigned(line, index, magnitude_i32(value));
}

static uint8_t append_angle(char *line, uint8_t index, int32_t millidegrees)
{
    const uint32_t magnitude = magnitude_i32(millidegrees);

    index = append_char(line, index, (millidegrees < 0) ? '-' : '+');
    index = append_unsigned(line, index, magnitude / 1000U);
    index = append_char(line, index, '.');
    return append_unsigned(line, index, (magnitude % 1000U) / 100U);
}

static uint8_t start_imu(ImuYaw *yaw)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < TEST_IMU_START_RETRIES; attempt++) {
        if (Icm42688_init()) {
            wait_ms(IMU_YAW_WARMUP_MS);
            if (ImuYaw_calibrateStationary(yaw)) {
                ImuYaw_rebaseClock(yaw);
                return 1U;
            }
        }
        if ((attempt + 1U) < TEST_IMU_START_RETRIES) {
            wait_ms(TEST_IMU_RETRY_GAP_MS);
        }
    }
    return 0U;
}

static void build_ready_lines(char lines[OLED_TEXT_LINE_COUNT]
                              [OLED_TEXT_LINE_MAX_CHARS + 1U])
{
    uint8_t line;

    for (line = 0U; line < OLED_TEXT_LINE_COUNT; line++) {
        line_clear(lines[line]);
    }
    (void) append_text(lines[0], 0U, "MOTOR IMU TEST");
    (void) append_text(lines[1], 0U, "ICM CAL COMPLETE");
    (void) append_text(lines[2], 0U, "WHEELS LIFTED?");
    (void) append_text(lines[3], 0U, "BODY MUST BE FIXED");
    (void) append_text(lines[4], 0U, "6 X 3SEC PHASES");
    (void) append_text(lines[5], 0U, "RESULT HOLD 5SEC");
    (void) append_text(lines[6], 0U, "PRESS S2 START");
    (void) append_text(lines[7], 0U, "RST TO ABORT");
}

static void build_phase_lines(char lines[OLED_TEXT_LINE_COUNT]
                              [OLED_TEXT_LINE_MAX_CHARS + 1U],
                              const TestPhase *phase, uint8_t phaseIndex,
                              const PhaseRecord *record, const ImuYaw *yaw,
                              uint16_t phaseElapsedMs, uint8_t finished)
{
    int32_t deltaMdeg = yaw->headingMdeg - record->headingStartMdeg;
    int32_t average = 0;
    uint8_t line;
    uint8_t index;

    if (record->validSamples != 0U) {
        average = (int32_t) (record->correctedSum /
                             (int64_t) record->validSamples);
    }

    for (line = 0U; line < OLED_TEXT_LINE_COUNT; line++) {
        line_clear(lines[line]);
    }

    index = append_text(lines[0], 0U, finished != 0U ? "RESULT P" : "RUN P");
    index = append_unsigned(lines[0], index, phaseIndex);
    index = append_char(lines[0], index, ' ');
    index = append_text(lines[0], index, phase->name);
    index = append_text(lines[0], index, " ");
    index = append_unsigned(lines[0], index, phase->leftDuty);
    index = append_char(lines[0], index, '/');
    (void) append_unsigned(lines[0], index, phase->rightDuty);

    index = append_text(lines[1], 0U, "DHDG");
    (void) append_angle(lines[1], index, deltaMdeg);

    index = append_text(lines[2], 0U, "AVG");
    index = append_signed(lines[2], index, average);
    index = append_text(lines[2], index, " PK");
    (void) append_unsigned(lines[2], index, record->correctedPeak);

    index = append_text(lines[3], 0U, "ENC");
    index = append_signed(lines[3], index,
                          g_motorA_encoder_count - record->leftEncoderStart);
    index = append_char(lines[3], index, '/');
    (void) append_signed(lines[3], index,
                          g_motorB_encoder_count - record->rightEncoderStart);

    index = append_text(lines[4], 0U, "BIAS");
    index = append_signed(lines[4], index, g_motor_imu_base_bias_raw);
    index = append_text(lines[4], index, " F");
    (void) append_unsigned(lines[4], index, yaw->failures);

    index = append_text(lines[5], 0U, finished != 0U ? "HOLD 5 SEC" : "TIME ");
    if (finished == 0U) {
        index = append_unsigned(lines[5], index, phaseElapsedMs / 1000U);
        index = append_char(lines[5], index, '.');
        (void) append_unsigned(lines[5], index,
                                (phaseElapsedMs % 1000U) / 100U);
    }
    (void) append_text(lines[6], 0U, "DHDG=FALSE YAW");
    (void) append_text(lines[7], 0U, "BODY MUST NOT TURN");
}

static void update_record(ImuYaw *yaw, PhaseRecord *record,
                          uint8_t allowBiasUpdate)
{
    int32_t fixedBiasCorrected;
    uint32_t magnitude;

    if (ImuYaw_update(yaw, allowBiasUpdate != 0U)) {
        fixedBiasCorrected = (int32_t) yaw->rawGyroZ -
                             g_motor_imu_base_bias_raw;
        magnitude = magnitude_i32(fixedBiasCorrected);
        record->correctedSum += fixedBiasCorrected;
        if (magnitude > record->correctedPeak) {
            record->correctedPeak = magnitude;
        }
        record->validSamples++;
        g_motor_imu_raw_z = yaw->rawGyroZ;
        g_motor_imu_corrected_raw = fixedBiasCorrected;
    }
    g_motor_imu_failures = yaw->failures;
}

static void wait_for_start(void)
{
    char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U];
    uint16_t pressedMs = 0U;

    build_ready_lines(lines);
    while (button_pressed() != 0U) {
        Oled_updateTextLines(lines, TEST_TICK_MS);
        wait_ms(TEST_TICK_MS);
    }
    while (pressedMs < TEST_BUTTON_DEBOUNCE_MS) {
        if (button_pressed() != 0U) {
            pressedMs = (uint16_t) (pressedMs + TEST_TICK_MS);
        } else {
            pressedMs = 0U;
        }
        Oled_updateTextLines(lines, TEST_TICK_MS);
        wait_ms(TEST_TICK_MS);
    }
}

int main(void)
{
    ImuYaw yaw = {0};
    char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U];
    uint8_t phaseIndex;

    Motor_init();
    Encoder_init();
    button_init();
    Motor_stopAll();
    systick_init();
    (void) Oled_init();

    if (start_imu(&yaw) == 0U) {
        build_ready_lines(lines);
        line_clear(lines[1]);
        (void) append_text(lines[1], 0U, "IMU CAL FAILED");
        line_clear(lines[6]);
        (void) append_text(lines[6], 0U, "CHECK ICM WIRING");
        for (;;) {
            Motor_stopAll();
            Oled_updateTextLines(lines, TEST_TICK_MS);
            wait_ms(TEST_TICK_MS);
        }
    }

    g_motor_imu_base_bias_raw = yaw.biasRaw;
    wait_for_start();
    Encoder_resetCounts();

    for (phaseIndex = 0U; phaseIndex < TEST_PHASE_COUNT; phaseIndex++) {
        const TestPhase *phase = &k_test_phases[phaseIndex];
        PhaseRecord record = {0};
        uint16_t phaseElapsedMs;

        record.headingStartMdeg = yaw.headingMdeg;
        record.leftEncoderStart = g_motorA_encoder_count;
        record.rightEncoderStart = g_motorB_encoder_count;
        g_motor_imu_phase = phaseIndex;
        g_motor_imu_left_duty = phase->leftDuty;
        g_motor_imu_right_duty = phase->rightDuty;

        for (phaseElapsedMs = 0U; phaseElapsedMs < TEST_PHASE_MS;
             phaseElapsedMs = (uint16_t) (phaseElapsedMs + TEST_TICK_MS)) {
            if ((phase->leftDuty != 0U) || (phase->rightDuty != 0U)) {
                Motor_runBothWithDutyFor(MOTOR_DIRECTION_FORWARD,
                                         MOTOR_DIRECTION_FORWARD,
                                         phase->leftDuty, phase->rightDuty,
                                         TEST_TICK_MS);
            } else {
                Motor_stopAll();
                Encoder_sample();
                wait_ms(TEST_TICK_MS);
            }
            update_record(&yaw, &record,
                ((phase->leftDuty == 0U) &&
                 (phase->rightDuty == 0U)) ? 1U : 0U);
            build_phase_lines(lines, phase, phaseIndex, &record, &yaw,
                              phaseElapsedMs, 0U);
            Oled_updateTextLines(lines, TEST_TICK_MS);
        }

        Motor_stopAll();
        g_motor_imu_phase_delta_mdeg[phaseIndex] =
            yaw.headingMdeg - record.headingStartMdeg;
        g_motor_imu_average_raw[phaseIndex] =
            (record.validSamples != 0U) ?
            (int32_t) (record.correctedSum / (int64_t) record.validSamples) : 0;
        g_motor_imu_peak_raw[phaseIndex] = record.correctedPeak;
        g_motor_imu_left_encoder_delta[phaseIndex] =
            g_motorA_encoder_count - record.leftEncoderStart;
        g_motor_imu_right_encoder_delta[phaseIndex] =
            g_motorB_encoder_count - record.rightEncoderStart;
        g_motor_imu_valid_samples[phaseIndex] = record.validSamples;

        build_phase_lines(lines, phase, phaseIndex, &record, &yaw,
                          TEST_PHASE_MS, 1U);
        for (phaseElapsedMs = 0U; phaseElapsedMs < TEST_RESULT_HOLD_MS;
             phaseElapsedMs = (uint16_t) (phaseElapsedMs + TEST_TICK_MS)) {
            Oled_updateTextLines(lines, TEST_TICK_MS);
            wait_ms(TEST_TICK_MS);
        }
    }

    line_clear(lines[0]);
    line_clear(lines[1]);
    line_clear(lines[2]);
    line_clear(lines[3]);
    line_clear(lines[4]);
    line_clear(lines[5]);
    line_clear(lines[6]);
    line_clear(lines[7]);
    (void) append_text(lines[0], 0U, "TEST COMPLETE");
    (void) append_text(lines[1], 0U, "RESULTS IN CCS WATCH");
    (void) append_text(lines[2], 0U, "P0..P5 RECORDED");
    (void) append_text(lines[3], 0U, "SEND OLED PHOTOS");
    (void) append_text(lines[4], 0U, "MOTORS RELEASED");
    (void) append_text(lines[5], 0U, "RST TO RUN AGAIN");
    for (;;) {
        Motor_stopAll();
        Oled_updateTextLines(lines, TEST_TICK_MS);
        wait_ms(TEST_TICK_MS);
    }
}
