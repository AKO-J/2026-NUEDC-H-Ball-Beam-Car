#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#include "competition_entries.h"
#include "competition_ui.h"
#include "motor_driver.h"
#include "ssd1306_oled.h"
#include "stepper_beam.h"

#define SELECTOR_PORT GPIOB
#define SELECTOR_S1_PIN DL_GPIO_PIN_7
#define SELECTOR_S2_PIN DL_GPIO_PIN_8
#define SELECTOR_S3_PIN DL_GPIO_PIN_15
#define SELECTOR_S4_PIN DL_GPIO_PIN_0
#define SELECTOR_S5_PIN DL_GPIO_PIN_20
#define SELECTOR_ALL_PINS (SELECTOR_S1_PIN | SELECTOR_S2_PIN | \
                           SELECTOR_S3_PIN | SELECTOR_S4_PIN | \
                           SELECTOR_S5_PIN)
#define SELECTOR_DEBOUNCE_TICKS 3U
#define SELECTOR_TICK_MS 10U

typedef struct {
    uint32_t stable;
    uint32_t candidate;
    uint8_t ticks;
} SelectorFilter;

static void wait_ms(uint32_t ms)
{
    while (ms-- != 0U) delay_cycles(32000U);
}

static void init_input(uint32_t iomux)
{
    DL_GPIO_initDigitalInputFeatures(iomux,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

static void selector_buttons_init(void)
{
    DL_GPIO_enablePower(SELECTOR_PORT);
    init_input(IOMUX_PINCM24); /* PB7: S1 */
    init_input(IOMUX_PINCM25); /* PB8: S2 */
    init_input(IOMUX_PINCM32); /* PB15: S3 */
    init_input(IOMUX_PINCM12); /* PB0: S4 */
    init_input(IOMUX_PINCM48); /* PB20: S5 reserved */
}

static uint32_t raw_buttons(void)
{
    return (~DL_GPIO_readPins(SELECTOR_PORT, SELECTOR_ALL_PINS)) &
           SELECTOR_ALL_PINS;
}

static uint32_t update_filter(SelectorFilter *filter, uint32_t sample)
{
    uint32_t edge = 0U;
    if (sample != filter->candidate) {
        filter->candidate = sample;
        filter->ticks = 0U;
    } else if (filter->ticks < SELECTOR_DEBOUNCE_TICKS) {
        ++filter->ticks;
        if ((filter->ticks == SELECTOR_DEBOUNCE_TICKS) &&
            (filter->stable != sample)) {
            edge = sample & ~filter->stable;
            filter->stable = sample;
        }
    }
    return edge;
}

static uint32_t ui_keys(uint32_t gpio)
{
    uint32_t keys = 0U;
    if ((gpio & SELECTOR_S1_PIN) != 0U) keys |= COMPETITION_KEY_START;
    if ((gpio & SELECTOR_S2_PIN) != 0U) keys |= COMPETITION_KEY_FUNCTION2;
    if ((gpio & SELECTOR_S3_PIN) != 0U) keys |= COMPETITION_KEY_FUNCTION3;
    if ((gpio & SELECTOR_S4_PIN) != 0U) keys |= COMPETITION_KEY_FUNCTION456;
    if ((gpio & SELECTOR_S5_PIN) != 0U) keys |= COMPETITION_KEY_RESERVED;
    return keys;
}

static void set_line(
    char line[OLED_TEXT_LINE_MAX_CHARS + 1U], const char *text)
{
    uint8_t index = 0U;
    while ((text[index] != '\0') &&
           (index < OLED_TEXT_LINE_MAX_CHARS)) {
        line[index] = text[index];
        ++index;
    }
    line[index] = '\0';
}

static void show_selector(const CompetitionUi *ui, uint32_t nowMs)
{
    char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U] = {{0}};
    set_line(lines[0], "SMARTCAR SELECT");
    set_line(lines[1], "S2: TASK 2 LAP");
    set_line(lines[2], "S3: TASK 3 +/-5");
    set_line(lines[3], "S4: TASK 4/5/6");
    set_line(lines[4], "S5: RESERVED");
    if (ui->selectedFunction == COMPETITION_FUNCTION_NONE) {
        set_line(lines[6], "NEXT: PRESS S2/S3/S4");
    } else if (ui->selectedFunction == COMPETITION_FUNCTION_TASK2) {
        set_line(lines[5], "SELECTED: TASK 2");
        set_line(lines[6], "NEXT: S1 ENTER");
    } else if (ui->selectedFunction == COMPETITION_FUNCTION_TASK3) {
        set_line(lines[5], "SELECTED: TASK 3");
        set_line(lines[6], "NEXT: S1 ENTER");
    } else {
        set_line(lines[5], "SELECTED: TASK 4/5/6");
        set_line(lines[6], "NEXT: S1 ENTER");
        set_line(lines[7], "THEN FOLLOW 4X S1");
    }
    Oled_updateTextLines(lines, (uint16_t) nowMs);
}

static void show_entering(CompetitionFunction function, uint32_t nowMs)
{
    char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U] = {{0}};
    if (function == COMPETITION_FUNCTION_TASK2)
        set_line(lines[0], "ENTER TASK 2");
    else if (function == COMPETITION_FUNCTION_TASK3)
        set_line(lines[0], "ENTER TASK 3");
    else
        set_line(lines[0], "ENTER TASK 4/5/6");
    set_line(lines[2], "RELEASE S1");
    set_line(lines[4], "MOTORS STILL OFF");
    set_line(lines[6], "FOLLOW TASK OLED");
    Oled_updateTextLines(lines, (uint16_t) nowMs);
}

static void wait_for_s1_release(uint32_t *nowMs)
{
    uint32_t releasedMs = 0U;
    while (releasedMs < 30U) {
        if ((raw_buttons() & SELECTOR_S1_PIN) == 0U)
            releasedMs += SELECTOR_TICK_MS;
        else
            releasedMs = 0U;
        wait_ms(SELECTOR_TICK_MS);
        *nowMs += SELECTOR_TICK_MS;
    }
}

int main(void)
{
    CompetitionUi ui;
    SelectorFilter filter = {0U, 0U, 0U};
    uint32_t nowMs = 0U;

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    Motor_init();
    Motor_stopAll();
    StepperBeam_init();
    StepperBeam_stop();
    StepperBeam_setArmed(false);
    selector_buttons_init();
    filter.stable = raw_buttons();
    filter.candidate = filter.stable;
    CompetitionUi_init(&ui);
    wait_ms(200U);
    (void) Oled_init();

    for (;;) {
        const uint32_t sample = raw_buttons();
        const uint32_t edge = update_filter(&filter, sample);
        const CompetitionUiAction action = CompetitionUi_update(
            &ui, ui_keys(edge), ui_keys(filter.stable));
        show_selector(&ui, nowMs);

        if (action == COMPETITION_ACTION_START_TASK) {
            const CompetitionFunction selected = ui.selectedFunction;
            show_entering(selected, nowMs);
            wait_for_s1_release(&nowMs);
            if (selected == COMPETITION_FUNCTION_TASK2)
                Task2_run();
            else if (selected == COMPETITION_FUNCTION_TASK3)
                Task3_run();
            else if (selected == COMPETITION_FUNCTION_TASK456)
                Joint456_run();
            for (;;) wait_ms(1000U);
        }

        wait_ms(SELECTOR_TICK_MS);
        nowMs += SELECTOR_TICK_MS;
    }
}
