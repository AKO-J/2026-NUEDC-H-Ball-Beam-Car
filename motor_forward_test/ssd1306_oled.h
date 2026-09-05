#ifndef SSD1306_OLED_H
#define SSD1306_OLED_H

#include <stdbool.h>
#include <stdint.h>

#define OLED_TEXT_LINE_COUNT 8U
#define OLED_TEXT_LINE_MAX_CHARS 21U

/*
 * The OLED uses an independent exposed header pair as software I2C:
 *   PB19 -> SCL (IOMUX_PINCM45)
 *   PA15 -> SDA (IOMUX_PINCM37)
 *
 * The ICM-42688 therefore remains on PA31=SCL and PA28=SDA.
 * PA18 must not be used for OLED SDA on this LaunchPad. It is the BSL invoke
 * pin and the OLED pull-up would prevent a normal cold boot.
 * PB13 must not be used either: it is the X42S STP pulse output.
 */

typedef struct {
    uint8_t functionId;
    int32_t headingMdeg;
    int32_t straightTargetMdeg;
    int32_t exitTargetMdeg;
    int32_t activeTargetMdeg;
    int32_t gyroBias;
    int32_t gyroZCorrected;
    uint32_t gyroSampleCycles;
    int16_t lineError;
    int16_t gyroZRaw;
    uint16_t traceCount;
    uint16_t whiteConfirmMs;
    uint16_t controlElapsedMs;
    uint8_t blackMask;
    uint8_t lineCandidateMask;
    uint16_t lineConfirmMs;
    uint8_t leftPwm;
    uint8_t rightPwm;
    uint8_t arcPhase;
    uint8_t arcNumber;
    int8_t arcDirection;
    uint8_t fixedProfile;
    uint8_t fixedInnerDuty;
    uint8_t fixedOuterDuty;
    uint8_t driveCommand;
    uint8_t imuHealthy;
    uint8_t imuFailures;
    uint8_t imuWhoAmI;
    uint8_t imuLastError;
    int32_t leftEncoder;
    int32_t rightEncoder;
    uint32_t runElapsedMs;
} OledStatus;

/* Returns false when no SSD1306 is found at 0x3C or 0x3D. */
bool Oled_init(void);

/* Store the current car status. The software-I2C bus sends one 32-column
 * chunk at most every 40 ms, so this is safe to call once per control slice.
 * A complete 128-column text page takes four chunks. */
void Oled_updateStatus(const OledStatus *status, uint16_t elapsedMs);

/* Show eight short diagnostic lines instead of the normal car telemetry.
 * Each line is limited to 21 ASCII characters (the width of a 128-pixel
 * display using the built-in 5x7 font). This is intended for isolated
 * hardware tests such as keys, without changing the normal car UI. */
void Oled_updateTextLines(
    const char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U],
    uint16_t elapsedMs);

/* CCS-watchable OLED diagnostics. */
extern volatile uint8_t g_oled_ready;
extern volatile uint8_t g_oled_i2c_address;
extern volatile uint8_t g_oled_last_error;
extern volatile uint8_t g_oled_last_page;
extern volatile uint32_t g_oled_refresh_count;

#endif
