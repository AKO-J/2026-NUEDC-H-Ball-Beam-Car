#include "icm42688.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/*
 * Use the free I2C0-capable GPIO pins already wired from the ICM board:
 *   PA28 -> SDA
 *   PA31 -> SCL
 * These pins do not overlap with the motor, encoder, or gray-sensor wiring.
 */
#define ICM_I2C_INST             I2C0
#define ICM_I2C_SDA_IOMUX        IOMUX_PINCM3
#define ICM_I2C_SCL_IOMUX        IOMUX_PINCM6
#define ICM_I2C_SDA_FUNCTION     IOMUX_PINCM3_PF_I2C0_SDA
#define ICM_I2C_SCL_FUNCTION     IOMUX_PINCM6_PF_I2C0_SCL
#define ICM_I2C_ADDRESS_LOW       0x68U
#define ICM_I2C_ADDRESS_HIGH      0x69U

#define ICM_REG_TEMP_DATA1       0x1DU
#define ICM_REG_GYRO_DATA_Z1     0x29U
#define ICM_REG_PWR_MGMT0        0x4EU
#define ICM_REG_GYRO_CONFIG0     0x4FU
#define ICM_REG_WHO_AM_I         0x75U
#define ICM_PWR_ACCEL_GYRO_LN    0x0FU
/* GYRO_FS_SEL=010 (+/-500 dps), GYRO_ODR=0111 (200 Hz). */
#define ICM_GYRO_500DPS_200HZ    0x47U

/* Generous limit: keeps a disconnected sensor from hanging the test program. */
#define ICM_I2C_TIMEOUT_LOOPS    100000U

/* MSPM0 I2C_ERR_13 timing and repeated-start transfer events. */
#define ICM_I2C_START_DELAY_CYCLES          8U
#define ICM_I2C_REPEATED_START_DELAY_CYCLES 1000U
#define ICM_I2C_RECOVERY_DELAY_CYCLES        3200U
#define ICM_I2C_TX_EVENTS \
    (DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | DL_I2C_INTERRUPT_CONTROLLER_NACK)
#define ICM_I2C_RX_EVENTS \
    (DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | DL_I2C_INTERRUPT_CONTROLLER_NACK)

enum {
    ICM_ERROR_NONE = 0U,
    ICM_ERROR_TIMEOUT = 1U,
    ICM_ERROR_BUS = 2U,
    ICM_ERROR_WHO_AM_I = 3U,
    ICM_ERROR_CONFIG = 4U,
};

volatile uint8_t g_icm42688_last_error;
volatile uint8_t g_icm42688_who_am_i;
volatile uint8_t g_icm42688_last_stage;
volatile uint8_t g_icm42688_i2c_address;
volatile uint8_t g_icm42688_sda_line_high;
volatile uint8_t g_icm42688_scl_line_high;
volatile uint8_t g_icm42688_gyro_config0;
volatile uint32_t g_icm42688_controller_status;

static uint8_t s_icm_i2c_address = ICM_I2C_ADDRESS_LOW;

static void i2c_init(void);

static void wait_ms(uint32_t milliseconds)
{
    while (milliseconds-- != 0U) {
        delay_cycles(32000U);
    }
}

static void recover_bus(void)
{
    /* A transfer-only reset does not clear every latched controller error on
     * MSPM0 I2C. Rebuild I2C0 so a one-off error cannot permanently disable
     * heading sampling after the stationary startup calibration. */
    i2c_init();
    delay_cycles(ICM_I2C_RECOVERY_DELAY_CYCLES);
}

static bool wait_transfer_idle(void)
{
    uint32_t timeout = ICM_I2C_TIMEOUT_LOOPS;

    while (timeout-- != 0U) {
        const uint32_t status = DL_I2C_getControllerStatus(ICM_I2C_INST);

        g_icm42688_controller_status = status;

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            g_icm42688_last_error = ICM_ERROR_BUS;
            recover_bus();
            return false;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return true;
        }
    }

    g_icm42688_last_error = ICM_ERROR_TIMEOUT;
    recover_bus();
    return false;
}

static bool wait_transfer_complete(void)
{
    uint32_t timeout = ICM_I2C_TIMEOUT_LOOPS;

    while (timeout-- != 0U) {
        const uint32_t status = DL_I2C_getControllerStatus(ICM_I2C_INST);

        g_icm42688_controller_status = status;

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            g_icm42688_last_error = ICM_ERROR_BUS;
            recover_bus();
            return false;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
            return true;
        }
    }

    g_icm42688_last_error = ICM_ERROR_TIMEOUT;
    recover_bus();
    return false;
}

/*
 * The first leg of a repeated-start read has STOP disabled. The controller
 * correctly remains BUSY while it owns the bus, so wait for TX_DONE instead
 * of waiting for BUSY to clear.
 */
static bool wait_tx_done_without_stop(void)
{
    uint32_t timeout = ICM_I2C_TIMEOUT_LOOPS;

    while (timeout-- != 0U) {
        const uint32_t status = DL_I2C_getControllerStatus(ICM_I2C_INST);
        const uint32_t events = DL_I2C_getRawInterruptStatus(
            ICM_I2C_INST, ICM_I2C_TX_EVENTS);

        g_icm42688_controller_status = status;

        if ((events & DL_I2C_INTERRUPT_CONTROLLER_NACK) != 0U ||
            (status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            g_icm42688_last_error = ICM_ERROR_BUS;
            recover_bus();
            return false;
        }
        if ((events & DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U) {
            DL_I2C_clearInterruptStatus(
                ICM_I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
            return true;
        }
    }

    g_icm42688_last_error = ICM_ERROR_TIMEOUT;
    recover_bus();
    return false;
}

static bool write_register(uint8_t reg, uint8_t value)
{
    uint8_t packet[2] = {reg, value};

    g_icm42688_last_stage = 4U;
    if (!wait_transfer_idle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(ICM_I2C_INST);
    (void) DL_I2C_fillControllerTXFIFO(ICM_I2C_INST, packet, sizeof(packet));
    DL_I2C_startControllerTransfer(ICM_I2C_INST, s_icm_i2c_address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   sizeof(packet));
    delay_cycles(ICM_I2C_START_DELAY_CYCLES);
    g_icm42688_last_stage = 5U;
    return wait_transfer_complete();
}

static bool read_registers(uint8_t startReg, uint8_t *data, uint8_t length)
{
    uint8_t received = 0U;
    uint32_t timeout;

    g_icm42688_last_stage = 1U;
    if ((data == 0) || (length == 0U) || !wait_transfer_idle()) {
        return false;
    }

    /* Write the register pointer without STOP, then issue the I2C repeated START. */
    DL_I2C_clearInterruptStatus(ICM_I2C_INST, ICM_I2C_TX_EVENTS);
    DL_I2C_flushControllerTXFIFO(ICM_I2C_INST);
    (void) DL_I2C_fillControllerTXFIFO(ICM_I2C_INST, &startReg, 1U);
    DL_I2C_startControllerTransferAdvanced(
        ICM_I2C_INST, s_icm_i2c_address, DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(ICM_I2C_START_DELAY_CYCLES);
    g_icm42688_last_stage = 2U;
    if (!wait_tx_done_without_stop()) {
        return false;
    }

    delay_cycles(ICM_I2C_REPEATED_START_DELAY_CYCLES);
    DL_I2C_clearInterruptStatus(ICM_I2C_INST, ICM_I2C_RX_EVENTS);
    DL_I2C_flushControllerRXFIFO(ICM_I2C_INST);
    DL_I2C_startControllerTransferAdvanced(
        ICM_I2C_INST, s_icm_i2c_address, DL_I2C_CONTROLLER_DIRECTION_RX, length,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(ICM_I2C_START_DELAY_CYCLES);
    g_icm42688_last_stage = 3U;

    while (received < length) {
        timeout = ICM_I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(ICM_I2C_INST)) {
            const uint32_t status = DL_I2C_getControllerStatus(ICM_I2C_INST);

            g_icm42688_controller_status = status;

            if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                g_icm42688_last_error = ICM_ERROR_BUS;
                recover_bus();
                return false;
            }
            if (timeout-- == 0U) {
                g_icm42688_last_error = ICM_ERROR_TIMEOUT;
                recover_bus();
                return false;
            }
        }
        data[received++] = DL_I2C_receiveControllerData(ICM_I2C_INST);
    }

    return wait_transfer_complete();
}

static int16_t join_big_endian(uint8_t high, uint8_t low)
{
    return (int16_t) (((uint16_t) high << 8U) | (uint16_t) low);
}

static void i2c_init(void)
{
    static const DL_I2C_ClockConfig clockConfig = {
        .clockSel = DL_I2C_CLOCK_BUSCLK,
        .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
    };

    DL_GPIO_enablePower(GPIOA);
    DL_I2C_reset(ICM_I2C_INST);
    DL_I2C_enablePower(ICM_I2C_INST);
    delay_cycles(16U);

    DL_GPIO_initPeripheralInputFunctionFeatures(
        ICM_I2C_SDA_IOMUX, ICM_I2C_SDA_FUNCTION, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        ICM_I2C_SCL_IOMUX, ICM_I2C_SCL_FUNCTION, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(ICM_I2C_SDA_IOMUX);
    DL_GPIO_enableHiZ(ICM_I2C_SCL_IOMUX);

    DL_I2C_setClockConfig(ICM_I2C_INST, (DL_I2C_ClockConfig *) &clockConfig);
    DL_I2C_disableAnalogGlitchFilter(ICM_I2C_INST);
    DL_I2C_resetControllerTransfer(ICM_I2C_INST);
    /* 100 kHz is deliberately conservative for loose breadboard jumpers. */
    DL_I2C_setTimerPeriod(ICM_I2C_INST, 31U);
    DL_I2C_setControllerTXFIFOThreshold(
        ICM_I2C_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(
        ICM_I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(ICM_I2C_INST);
    DL_I2C_enableController(ICM_I2C_INST);
}

bool Icm42688_init(void)
{
    uint8_t gyroConfig0;
    uint8_t whoAmI;

    g_icm42688_last_error = ICM_ERROR_NONE;
    g_icm42688_who_am_i = 0U;
    g_icm42688_last_stage = 0U;
    g_icm42688_gyro_config0 = 0U;
    g_icm42688_controller_status = 0U;
    s_icm_i2c_address = ICM_I2C_ADDRESS_LOW;
    g_icm42688_i2c_address = s_icm_i2c_address;
    i2c_init();
    g_icm42688_sda_line_high =
        ((DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_28) & DL_GPIO_PIN_28) != 0U) ? 1U : 0U;
    g_icm42688_scl_line_high =
        ((DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_31) & DL_GPIO_PIN_31) != 0U) ? 1U : 0U;
    wait_ms(20U); /* Sensor board and I2C pullups have settled after power-up. */

    if (!read_registers(ICM_REG_WHO_AM_I, &whoAmI, 1U)) {
        /* AD0 is commonly strapped either low (0x68) or high (0x69). */
        s_icm_i2c_address = ICM_I2C_ADDRESS_HIGH;
        g_icm42688_i2c_address = s_icm_i2c_address;
        g_icm42688_last_error = ICM_ERROR_NONE;
        if (!read_registers(ICM_REG_WHO_AM_I, &whoAmI, 1U)) {
            return false;
        }
    }
    g_icm42688_who_am_i = whoAmI;
    g_icm42688_last_stage = 6U;
    if (whoAmI != ICM42688_WHO_AM_I_EXPECTED) {
        g_icm42688_last_error = ICM_ERROR_WHO_AM_I;
        return false;
    }

    if (!write_register(ICM_REG_PWR_MGMT0, ICM_PWR_ACCEL_GYRO_LN)) {
        return false;
    }
    wait_ms(50U); /* Datasheet startup allowance for accel and gyro low-noise modes. */

    if (!write_register(ICM_REG_GYRO_CONFIG0, ICM_GYRO_500DPS_200HZ) ||
        !read_registers(ICM_REG_GYRO_CONFIG0, &gyroConfig0, 1U)) {
        return false;
    }
    g_icm42688_gyro_config0 = gyroConfig0;
    if (gyroConfig0 != ICM_GYRO_500DPS_200HZ) {
        g_icm42688_last_error = ICM_ERROR_CONFIG;
        return false;
    }
    g_icm42688_last_error = ICM_ERROR_NONE;
    return true;
}

bool Icm42688_readSample(Icm42688Sample *sample)
{
    uint8_t raw[14];

    if (sample == 0) {
        return false;
    }
    if (!read_registers(ICM_REG_TEMP_DATA1, raw, sizeof(raw))) {
        return false;
    }

    sample->temperature = join_big_endian(raw[0], raw[1]);
    sample->accelX = join_big_endian(raw[2], raw[3]);
    sample->accelY = join_big_endian(raw[4], raw[5]);
    sample->accelZ = join_big_endian(raw[6], raw[7]);
    sample->gyroX = join_big_endian(raw[8], raw[9]);
    sample->gyroY = join_big_endian(raw[10], raw[11]);
    sample->gyroZ = join_big_endian(raw[12], raw[13]);
    g_icm42688_last_error = ICM_ERROR_NONE;
    return true;
}

bool Icm42688_readGyroZ(int16_t *gyroZ)
{
    uint8_t raw[2];

    if (gyroZ == 0) {
        return false;
    }
    if (!read_registers(ICM_REG_GYRO_DATA_Z1, raw, sizeof(raw))) {
        return false;
    }

    *gyroZ = join_big_endian(raw[0], raw[1]);
    g_icm42688_last_error = ICM_ERROR_NONE;
    return true;
}
