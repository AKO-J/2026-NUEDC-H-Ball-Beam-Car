#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "competition_ui.h"
#include "ssd1306_oled.h"

#define KEY_PORT GPIOB
#define KEY_START_PIN DL_GPIO_PIN_7
#define KEY_TASK2_PIN DL_GPIO_PIN_8
#define KEY_TASK3_PIN DL_GPIO_PIN_15
#define KEY_TASK4_PIN DL_GPIO_PIN_0
#define KEY_TASK5_PIN DL_GPIO_PIN_20
#define KEY_ALL_PINS (KEY_START_PIN | KEY_TASK2_PIN | KEY_TASK3_PIN | \
                      KEY_TASK4_PIN | KEY_TASK5_PIN)
#define DEBOUNCE_TICKS 3U

typedef struct {
    uint32_t stable;
    uint32_t candidate;
    uint8_t ticks;
} KeyFilter;

static void wait_ms(uint32_t ms)
{
    while (ms-- != 0U) delay_cycles(32000U);
}

static void init_key(uint32_t iomux)
{
    DL_GPIO_initDigitalInputFeatures(iomux,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

static void keys_init(void)
{
    DL_GPIO_enablePower(KEY_PORT);
    init_key(IOMUX_PINCM24); /* PB7: START/STOP */
    init_key(IOMUX_PINCM25); /* PB8: task 2 / CAL down */
    init_key(IOMUX_PINCM32); /* PB15: task 3 / CAL up */
    init_key(IOMUX_PINCM12); /* PB0: task 4 / CAL preset */
    init_key(IOMUX_PINCM48); /* PB20: task 5 / CAL save level */
}

static uint32_t raw_keys(void)
{
    return (~DL_GPIO_readPins(KEY_PORT, KEY_ALL_PINS)) & KEY_ALL_PINS;
}

static uint32_t key_edge(KeyFilter *filter, uint32_t sample)
{
    uint32_t edge = 0U;
    if (sample != filter->candidate) {
        filter->candidate = sample;
        filter->ticks = 0U;
    } else if (filter->ticks < DEBOUNCE_TICKS) {
        ++filter->ticks;
        if ((filter->ticks == DEBOUNCE_TICKS) &&
            (filter->stable != sample)) {
            edge = sample & ~filter->stable;
            filter->stable = sample;
        }
    }
    return edge;
}

static uint32_t to_ui_keys(uint32_t gpio)
{
    uint32_t keys = 0U;
    if ((gpio & KEY_START_PIN) != 0U) keys |= COMPETITION_KEY_START;
    if ((gpio & KEY_TASK2_PIN) != 0U) keys |= COMPETITION_KEY_FUNCTION2;
    if ((gpio & KEY_TASK3_PIN) != 0U) keys |= COMPETITION_KEY_FUNCTION3;
    if ((gpio & KEY_TASK4_PIN) != 0U) keys |= COMPETITION_KEY_FUNCTION456;
    if ((gpio & KEY_TASK5_PIN) != 0U) keys |= COMPETITION_KEY_RESERVED;
    return keys;
}

static void set_line(
    char dst[OLED_TEXT_LINE_MAX_CHARS + 1U], const char *src)
{
    uint8_t i = 0U;
    while ((src[i] != '\0') && (i < OLED_TEXT_LINE_MAX_CHARS)) {
        dst[i] = src[i];
        ++i;
    }
    dst[i] = '\0';
}

static void draw(const CompetitionUi *ui, CompetitionUiAction action,
                 uint32_t nowMs)
{
    char text[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U] = {{0}};

    if (ui->state == COMPETITION_UI_SELECT) {
        set_line(text[0], "SELECT TASK");
        set_line(text[1], "S2: TASK 2 LAP");
        set_line(text[2], "S3: TASK 3 +/-5");
        set_line(text[3], "S4: TASK 4/5/6");
        set_line(text[5], "S5: RESERVED");
        set_line(text[7], "S1: START/STOP");
    } else {
        if (ui->selectedFunction == COMPETITION_FUNCTION_TASK2)
            set_line(text[0], "FUNCTION: TASK 2");
        else if (ui->selectedFunction == COMPETITION_FUNCTION_TASK3)
            set_line(text[0], "FUNCTION: TASK 3");
        else
            set_line(text[0], "FUNCTION: TASK 4/5/6");
        if (ui->state == COMPETITION_UI_READY) {
            set_line(text[1], "SELECTED / SAFE");
            set_line(text[3], "S1: START REQUEST");
            set_line(text[5], "S2-S4: RESELECT");
            set_line(text[6], "S5: RESERVED");
        } else {
            set_line(text[1], "RUN REQUEST (TEST)");
            set_line(text[3], "MOTORS ARE DISABLED");
            set_line(text[6], "S1: EMERGENCY STOP");
        }
    }

    if (action == COMPETITION_ACTION_EMERGENCY_STOP)
        set_line(text[7], "STOP REQUESTED");

    Oled_updateTextLines(text, (uint16_t) nowMs);
}

int main(void)
{
    CompetitionUi ui;
    KeyFilter filter = {0U, 0U, 0U};
    uint32_t nowMs = 0U;

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    keys_init();
    filter.stable = raw_keys();
    filter.candidate = filter.stable;
    CompetitionUi_init(&ui);
    wait_ms(200U);
    (void) Oled_init();

    while (1) {
        const uint32_t sample = raw_keys();
        const uint32_t edge = key_edge(&filter, sample);
        const CompetitionUiAction action = CompetitionUi_update(
            &ui, to_ui_keys(edge), to_ui_keys(filter.stable));
        draw(&ui, action, nowMs);
        wait_ms(10U);
        nowMs += 10U;
    }
}
