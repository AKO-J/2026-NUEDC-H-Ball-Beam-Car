/*
 * LF04 first-power diagnostic. This image never initializes a motor output.
 * Watch the variables below in CCS while moving each probe between white
 * paper and the black tape. LED1 toggles every 500 ms as a heartbeat.
 */

#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "ir_line_sensor.h"
#include "ssd1306_oled.h"

#define HEARTBEAT_LED_PORT          GPIOA
#define HEARTBEAT_LED_PIN           DL_GPIO_PIN_0
#define HEARTBEAT_LED_IOMUX         IOMUX_PINCM1
#define TEST_SAMPLE_MS              5U
#define HEARTBEAT_TOGGLE_MS         500U
#define MCLK_CYCLES_PER_MS          32000U

volatile uint8_t g_ir_raw_mask;
volatile uint8_t g_ir_black_if_high_mask;
volatile uint8_t g_ir_black_if_low_mask;
volatile uint8_t g_ir_black_left_to_right_mask;
volatile uint8_t g_ir_dh_white_state;
volatile uint8_t g_ir_o1;
volatile uint8_t g_ir_o2;
volatile uint8_t g_ir_o3;
volatile uint8_t g_ir_o4;
volatile uint32_t g_ir_sample_count;
volatile uint8_t g_ir_oled_ready;

static char s_oled_lines[OLED_TEXT_LINE_COUNT]
                        [OLED_TEXT_LINE_MAX_CHARS + 1U] = {
    "LF04 READ TEST",
    "O1 O2 O3 O4",
    "RAW: 0 0 0 0",
    "BLACK:0 0 0 0",
    "BLACK = LOW (0)",
    "WHITE = HIGH (1)",
    "LED1 = HEARTBEAT",
    "NO MOTOR OUTPUT",
};

static void heartbeat_init(void)
{
    DL_GPIO_enablePower(HEARTBEAT_LED_PORT);
    delay_cycles(16U);
    DL_GPIO_initDigitalOutput(HEARTBEAT_LED_IOMUX);
    DL_GPIO_setPins(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
    DL_GPIO_enableOutput(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
}

static void update_oled(uint8_t rawMask)
{
    const uint8_t blackMask = IrLineSensor_rawToBlackMask(rawMask, 0U);
    uint8_t index;

    for (index = 0U; index < IR_LINE_SENSOR_CHANNEL_COUNT; index++) {
        const uint8_t position = (uint8_t) (5U + (index * 2U));

        s_oled_lines[2][position] =
            ((rawMask >> index) & 0x01U) != 0U ? '1' : '0';
        s_oled_lines[3][position + 1U] =
            ((blackMask >> index) & 0x01U) != 0U ? '1' : '0';
    }
    Oled_updateTextLines(s_oled_lines, TEST_SAMPLE_MS);
}

int main(void)
{
    uint16_t heartbeatElapsedMs = 0U;

    IrLineSensor_init();
    heartbeat_init();
    g_ir_oled_ready = Oled_init() ? 1U : 0U;

    for (;;) {
        const uint8_t rawMask = IrLineSensor_readStableRawMask();

        g_ir_raw_mask = rawMask;
        g_ir_black_if_high_mask =
            IrLineSensor_rawToBlackMask(rawMask, 1U);
        g_ir_black_if_low_mask =
            IrLineSensor_rawToBlackMask(rawMask, 0U);
        g_ir_black_left_to_right_mask = IrLineSensor_toLeftToRightMask(
            g_ir_black_if_low_mask, 0U);
        g_ir_dh_white_state =
            IrLineSensor_rawToDhWhiteState(rawMask, 0U);
        g_ir_o1 = (rawMask >> 0U) & 0x01U;
        g_ir_o2 = (rawMask >> 1U) & 0x01U;
        g_ir_o3 = (rawMask >> 2U) & 0x01U;
        g_ir_o4 = (rawMask >> 3U) & 0x01U;
        g_ir_sample_count++;
        update_oled(rawMask);

        heartbeatElapsedMs = (uint16_t) (heartbeatElapsedMs + TEST_SAMPLE_MS);
        if (heartbeatElapsedMs >= HEARTBEAT_TOGGLE_MS) {
            DL_GPIO_togglePins(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
            heartbeatElapsedMs = 0U;
        }
        delay_cycles(TEST_SAMPLE_MS * MCLK_CYCLES_PER_MS);
    }
}
