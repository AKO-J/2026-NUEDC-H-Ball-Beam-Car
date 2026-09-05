#include "diagnostic_uart.h"

#include <stddef.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

enum {
    DIAGNOSTIC_UART_IBRD_32_MHZ_115200 = 17,
    DIAGNOSTIC_UART_FBRD_32_MHZ_115200 = 23,
};

static void write_char(char character)
{
    DL_UART_Main_transmitDataBlocking(UART0, (uint8_t) character);
}

static void write_text(const char *text)
{
    if (text == NULL) {
        return;
    }
    while (*text != '\0') {
        write_char(*text++);
    }
}

static void write_uint32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char) ('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    while (count != 0U) {
        write_char(digits[--count]);
    }
}

static void write_int32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        write_char('-');
        /* Avoid signed overflow for INT32_MIN. */
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }
    write_uint32(magnitude);
}

static void write_comma(void)
{
    write_char(',');
}

void DiagnosticUart_init(void)
{
    static const DL_UART_Main_ClockConfig clockConfig = {
        .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
    };
    static const DL_UART_Main_Config uartConfig = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE,
    };

    /* PA10 is already wired to the LaunchPad XDS110 backchannel TX. */
    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    DL_UART_Main_reset(UART0);
    DL_UART_Main_enablePower(UART0);
    delay_cycles(16U);
    DL_UART_Main_setClockConfig(
        UART0, (DL_UART_Main_ClockConfig *) &clockConfig);
    DL_UART_Main_init(UART0, (DL_UART_Main_Config *) &uartConfig);
    DL_UART_Main_setOversampling(UART0, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        UART0,
        DIAGNOSTIC_UART_IBRD_32_MHZ_115200,
        DIAGNOSTIC_UART_FBRD_32_MHZ_115200);
    DL_UART_Main_enableFIFOs(UART0);
    DL_UART_Main_enable(UART0);
}

void DiagnosticUart_writeCsv(
    uint32_t pcMs,
    const VisionBallMeasurement *vision,
    int32_t ageMs,
    int16_t targetX,
    int32_t velocityPxPerS,
    int16_t errorPx,
    int16_t commandMdeg,
    const StepperBeamStatus *motor,
    const char *state)
{
    if ((vision == NULL) || (motor == NULL) || (state == NULL)) {
        return;
    }
    write_uint32(pcMs);
    write_comma();
    write_uint32(vision->frame);
    write_comma();
    write_uint32(vision->k230Ms);
    write_comma();
    write_int32(vision->xOffsetPx);
    write_comma();
    write_uint32(vision->confidenceMilli);
    write_comma();
    write_int32(ageMs);
    write_comma();
    write_int32(targetX);
    write_comma();
    write_int32(velocityPxPerS);
    write_comma();
    write_int32(errorPx);
    write_comma();
    write_int32(commandMdeg);
    write_comma();
    write_int32(motor->targetSteps);
    write_comma();
    write_int32(motor->positionSteps);
    write_comma();
    write_uint32(motor->pulseRatePps);
    write_comma();
    write_text(state);
    write_text("\r\n");
}

void DiagnosticUart_writeTask4Csv(
    uint32_t pcMs, const VisionBallMeasurement *vision, int32_t ballErrorCmX100,
    int32_t ageMs, int32_t velocityPxPerS, int16_t commandMdeg,
    const StepperBeamStatus *motor, int32_t leftEncoder, int32_t rightEncoder,
    int32_t leftSpeedCountsPerS, int32_t rightSpeedCountsPerS,
    int32_t accelXRaw, int32_t accelYRaw, int32_t accelZRaw,
    uint8_t leftPwm, uint8_t rightPwm, const char *phase,
    const char *safetyState)
{
    if ((vision == NULL) || (motor == NULL) || (phase == NULL) ||
        (safetyState == NULL)) {
        return;
    }
#define TASK4_FIELD(value) do { write_int32(value); write_comma(); } while (0)
    write_uint32(pcMs); write_comma();
    write_uint32(vision->frame); write_comma();
    write_uint32(vision->k230Ms); write_comma();
    TASK4_FIELD(vision->xOffsetPx);
    TASK4_FIELD(ballErrorCmX100);
    write_uint32(vision->confidenceMilli); write_comma();
    TASK4_FIELD(ageMs);
    TASK4_FIELD(velocityPxPerS);
    TASK4_FIELD(commandMdeg);
    TASK4_FIELD(motor->targetSteps);
    TASK4_FIELD(motor->positionSteps);
    write_uint32(motor->pulseRatePps); write_comma();
    TASK4_FIELD(leftEncoder);
    TASK4_FIELD(rightEncoder);
    TASK4_FIELD(leftSpeedCountsPerS);
    TASK4_FIELD(rightSpeedCountsPerS);
    TASK4_FIELD(accelXRaw);
    TASK4_FIELD(accelYRaw);
    TASK4_FIELD(accelZRaw);
    write_uint32(leftPwm); write_comma();
    write_uint32(rightPwm); write_comma();
    write_text(phase); write_comma();
    write_text(safetyState); write_text("\r\n");
#undef TASK4_FIELD
}

void DiagnosticUart_writeJointCsv(const DiagnosticJointTelemetry *t)
{
    if ((t == NULL) || (t->state == NULL) || (t->safetyState == NULL)) return;
#define JFIELD(value) do { write_int32(value); write_comma(); } while (0)
    write_uint32(t->mcuMs); write_comma(); write_uint32(t->runId); write_comma();
    write_text(t->state); write_comma(); write_text(t->safetyState); write_comma();
    write_uint32(t->irRawMask); write_comma();
    JFIELD(t->lineError); JFIELD(t->lineErrorRate); JFIELD(t->lineCorrection);
    JFIELD(t->baseSpeedRef); JFIELD(t->baseAccelerationRef);
    JFIELD(t->leftSpeedRef); JFIELD(t->rightSpeedRef);
    JFIELD(t->leftSpeed); JFIELD(t->rightSpeed);
    JFIELD(t->leftSpeedError); JFIELD(t->rightSpeedError);
    write_uint32(t->leftPwm); write_comma(); write_uint32(t->rightPwm); write_comma();
    JFIELD(t->leftEncoder); JFIELD(t->rightEncoder);
    JFIELD(t->vehicleSpeed); JFIELD(t->vehicleAcceleration);
    JFIELD(t->imuAccelX); JFIELD(t->imuAccelY); JFIELD(t->imuAccelZ);
    JFIELD(t->kffMilli); JFIELD(t->beamFeedforwardMdeg);
    JFIELD(t->beamBallPdMdeg); JFIELD(t->beamTargetMdeg);
    JFIELD(t->stepperTarget); JFIELD(t->stepperPosition);
    write_uint32(t->stepFrequency); write_comma(); write_uint32(t->visionFrame); write_comma();
    JFIELD(t->visionAgeMs); JFIELD(t->ballTargetCmX100); JFIELD(t->ballErrorCmX100);
    JFIELD(t->ballVelocityCmPerSX100);
    write_uint32(t->faultFlags); write_text("\r\n");
#undef JFIELD
}
