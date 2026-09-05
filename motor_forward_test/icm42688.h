#ifndef ICM42688_H
#define ICM42688_H

#include <stdbool.h>
#include <stdint.h>

/* ICM-42688-P reports this value from register WHO_AM_I (0x75). */
#define ICM42688_WHO_AM_I_EXPECTED 0x47U

typedef struct {
    int16_t temperature;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} Icm42688Sample;

/*
 * Initializes I2C0 on PA28(SDA) and PA31(SCL), checks WHO_AM_I, enables
 * low-noise mode, and verifies gyro +/-500 dps at 200 Hz. It never touches
 * motor GPIO.
 */
bool Icm42688_init(void);

/* Reads temperature, XYZ acceleration, and XYZ angular-rate raw values. */
bool Icm42688_readSample(Icm42688Sample *sample);

/* Fast yaw path: reads only GYRO_DATA_Z1/Z0 instead of the full 14-byte frame. */
bool Icm42688_readGyroZ(int16_t *gyroZ);

/* The most recent diagnostic result. 0 means the last operation succeeded. */
extern volatile uint8_t g_icm42688_last_error;
extern volatile uint8_t g_icm42688_who_am_i;
extern volatile uint8_t g_icm42688_last_stage;
extern volatile uint8_t g_icm42688_i2c_address;
extern volatile uint8_t g_icm42688_sda_line_high;
extern volatile uint8_t g_icm42688_scl_line_high;
extern volatile uint8_t g_icm42688_gyro_config0;
extern volatile uint32_t g_icm42688_controller_status;

#endif
