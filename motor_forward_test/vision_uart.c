#include "vision_uart.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

enum {
    VISION_UART_RX_CAPACITY = 128,
    VISION_UART_IBRD_32_MHZ_921600 = 2,
    VISION_UART_FBRD_32_MHZ_921600 = 11,
};

static volatile uint8_t s_rxBytes[VISION_UART_RX_CAPACITY];
static volatile uint8_t s_rxWrite;
static volatile uint8_t s_rxRead;
static volatile uint32_t s_droppedBytes;
static VisionBallParser s_parser;

void VisionUart_init(void)
{
    static const DL_UART_Main_ClockConfig clockConfig = {
        .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
    };
    static const DL_UART_Main_Config uartConfig = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE,
    };

    s_rxWrite = 0U;
    s_rxRead = 0U;
    s_droppedBytes = 0U;
    VisionBallParser_init(&s_parser);

    DL_GPIO_initPeripheralInputFunction(
        IOMUX_PINCM35, IOMUX_PINCM35_PF_UART3_RX);

    DL_UART_Main_reset(UART3);
    DL_UART_Main_enablePower(UART3);
    delay_cycles(16U);
    DL_UART_Main_setClockConfig(
        UART3, (DL_UART_Main_ClockConfig *) &clockConfig);
    DL_UART_Main_init(UART3, (DL_UART_Main_Config *) &uartConfig);
    DL_UART_Main_setOversampling(UART3, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        UART3,
        VISION_UART_IBRD_32_MHZ_921600,
        VISION_UART_FBRD_32_MHZ_921600);
    DL_UART_Main_enableFIFOs(UART3);
    DL_UART_Main_setRXFIFOThreshold(
        UART3, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(
        UART3,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    DL_UART_Main_setRXInterruptTimeout(UART3, 8U);
    NVIC_ClearPendingIRQ(UART3_INT_IRQn);
    NVIC_EnableIRQ(UART3_INT_IRQn);
    DL_UART_Main_enable(UART3);
}

bool VisionUart_pollNewest(VisionBallMeasurement *measurement)
{
    VisionBallMeasurement decoded;
    bool haveFrame = false;

    if (measurement == NULL) {
        return false;
    }
    while (s_rxRead != s_rxWrite) {
        const uint8_t byte = s_rxBytes[s_rxRead];
        s_rxRead = (uint8_t) ((s_rxRead + 1U) %
                             VISION_UART_RX_CAPACITY);
        if (VisionBallParser_feed(&s_parser, byte, &decoded)) {
            *measurement = decoded;
            haveFrame = true;
        }
    }
    return haveFrame;
}

uint32_t VisionUart_getAcceptedFrameCount(void)
{
    return s_parser.acceptedFrames;
}

uint32_t VisionUart_getRejectedFrameCount(void)
{
    return s_parser.rejectedFrames;
}

uint32_t VisionUart_getDroppedByteCount(void)
{
    return s_droppedBytes;
}

void UART3_IRQHandler(void)
{
    const DL_UART_IIDX cause =
        DL_UART_Main_getPendingInterrupt(UART3);

    if ((cause == DL_UART_MAIN_IIDX_RX) ||
        (cause == DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR)) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART3)) {
            const uint8_t byte = DL_UART_Main_receiveData(UART3);
            const uint8_t next =
                (uint8_t) ((s_rxWrite + 1U) % VISION_UART_RX_CAPACITY);
            if (next == s_rxRead) {
                ++s_droppedBytes;
            } else {
                s_rxBytes[s_rxWrite] = byte;
                s_rxWrite = next;
            }
        }
    }
}
