#include "ssd1306_oled.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* The OLED has its own, actually exposed GPIO pair: PB19=SCL and PA15=SDA.
 * The display deliberately uses open-drain software I2C here, so the
 * ICM-42688 can keep hardware I2C0 on PA31=SCL and PA28=SDA. PB13 is not an
 * alternative: it is the X42S STP pulse output. PA18 is also prohibited, as
 * it is the LaunchPad BSL-invoke input and an OLED SDA pull-up can make a real
 * power-on enter the ROM bootloader instead of this application.
 * Most 0.96-inch I2C OLED boards are 128 x 64 SSD1306 panels. For a 128 x 32
 * panel, change this value to 32U. */
#define OLED_DISPLAY_HEIGHT               64U
#define OLED_PAGE_COUNT                   (OLED_DISPLAY_HEIGHT / 8U)
#define OLED_PAGE_WIDTH                  128U
#define OLED_SCL_PORT                     GPIOB
#define OLED_SDA_PORT                     GPIOA
#define OLED_SCL_PIN                      DL_GPIO_PIN_19
#define OLED_SDA_PIN                      DL_GPIO_PIN_15
#define OLED_SCL_IOMUX                    IOMUX_PINCM45
#define OLED_SDA_IOMUX                    IOMUX_PINCM37
#define OLED_I2C_ADDRESS_DEFAULT          0x3CU
#define OLED_I2C_ADDRESS_ALTERNATE        0x3DU
#define OLED_SW_I2C_HALF_PERIOD_CYCLES    160U
#define OLED_SW_I2C_STRETCH_TIMEOUT       800U
/* A modestly faster status refresh: each 128-pixel page still transfers in
 * four small chunks, leaving ample time for the separate ICM I2C bus. */
#define OLED_REFRESH_INTERVAL_MS           30U
#define OLED_REFRESH_COLUMNS               32U
#define OLED_REFRESH_CHUNKS                (OLED_PAGE_WIDTH / OLED_REFRESH_COLUMNS)

enum {
    OLED_ERROR_NONE = 0U,
    OLED_ERROR_TRANSFER = 1U,
    OLED_ERROR_NOT_FOUND = 2U,
};

volatile uint8_t g_oled_ready;
volatile uint8_t g_oled_i2c_address;
volatile uint8_t g_oled_last_error;
volatile uint8_t g_oled_last_page;
volatile uint32_t g_oled_refresh_count;

typedef struct {
    char character;
    uint8_t columns[5];
} OledGlyph;

/* Compact 5 x 7 ASCII glyphs: enough for all telemetry labels and values. */
static const OledGlyph k_glyphs[] = {
    {' ', {0x00U, 0x00U, 0x00U, 0x00U, 0x00U}},
    {'0', {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}},
    {'1', {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}},
    {'2', {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}},
    {'3', {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}},
    {'4', {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}},
    {'5', {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}},
    {'6', {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}},
    {'7', {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}},
    {'8', {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'9', {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}},
    {'A', {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}},
    {'B', {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'C', {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}},
    {'D', {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}},
    {'E', {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}},
    {'F', {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}},
    {'G', {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}},
    {'H', {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}},
    {'I', {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}},
    {'J', {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}},
    {'K', {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}},
    {'L', {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}},
    {'M', {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}},
    {'N', {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}},
    {'O', {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}},
    {'P', {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}},
    {'Q', {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}},
    {'R', {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}},
    {'S', {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}},
    {'T', {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}},
    {'U', {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}},
    {'V', {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}},
    {'W', {0x7FU, 0x20U, 0x18U, 0x20U, 0x7FU}},
    {'X', {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}},
    {'Y', {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}},
    {'Z', {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}},
    {'+', {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U}},
    {'-', {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}},
    {'.', {0x00U, 0x60U, 0x60U, 0x00U, 0x00U}},
    {'/', {0x20U, 0x10U, 0x08U, 0x04U, 0x02U}},
    {'=', {0x14U, 0x14U, 0x14U, 0x14U, 0x14U}},
    {':', {0x00U, 0x36U, 0x36U, 0x00U, 0x00U}},
    {'?', {0x02U, 0x01U, 0x51U, 0x09U, 0x06U}},
};

static OledStatus s_status;
static char s_text_lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U];
static uint8_t s_text_mode;
static uint8_t s_i2c_address;
static uint8_t s_next_page;
static uint8_t s_next_chunk;
static uint16_t s_refresh_elapsed_ms;
static uint8_t s_page_data[OLED_PAGE_WIDTH];

static const uint8_t *glyph_for(char character)
{
    uint8_t index;

    if ((character >= 'a') && (character <= 'z')) {
        character = (char) (character - ('a' - 'A'));
    }
    for (index = 0U; index < (uint8_t) (sizeof(k_glyphs) / sizeof(k_glyphs[0]));
         index++) {
        if (k_glyphs[index].character == character) {
            return k_glyphs[index].columns;
        }
    }
    return k_glyphs[0].columns;
}

static void sw_i2c_delay(void)
{
    delay_cycles(OLED_SW_I2C_HALF_PERIOD_CYCLES);
}

/* The pins only ever actively drive low. Releasing the output lets the OLED's
 * 3.3-V pullups (and the weak MCU backup pullups) create an I2C-valid high. */
static void sda_low(void)
{
    DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN);
    DL_GPIO_enableOutput(OLED_SDA_PORT, OLED_SDA_PIN);
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(OLED_SDA_PORT, OLED_SDA_PIN);
}

static void scl_low(void)
{
    DL_GPIO_clearPins(OLED_SCL_PORT, OLED_SCL_PIN);
    DL_GPIO_enableOutput(OLED_SCL_PORT, OLED_SCL_PIN);
}

static bool scl_release_and_wait_high(void)
{
    uint16_t timeout = OLED_SW_I2C_STRETCH_TIMEOUT;

    DL_GPIO_disableOutput(OLED_SCL_PORT, OLED_SCL_PIN);
    while ((DL_GPIO_readPins(OLED_SCL_PORT, OLED_SCL_PIN) == 0U) &&
           (timeout-- != 0U)) {
        sw_i2c_delay();
    }
    return (timeout != 0U) ? true : false;
}

static void sw_i2c_start(void)
{
    sda_release();
    (void) scl_release_and_wait_high();
    sw_i2c_delay();
    sda_low();
    sw_i2c_delay();
    scl_low();
}

static void sw_i2c_stop(void)
{
    sda_low();
    sw_i2c_delay();
    (void) scl_release_and_wait_high();
    sw_i2c_delay();
    sda_release();
    sw_i2c_delay();
}

static bool sw_i2c_write_byte(uint8_t value)
{
    uint8_t bit;
    uint8_t acknowledged;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        sw_i2c_delay();
        if (!scl_release_and_wait_high()) {
            return false;
        }
        sw_i2c_delay();
        scl_low();
        value <<= 1U;
    }

    sda_release();
    sw_i2c_delay();
    if (!scl_release_and_wait_high()) {
        return false;
    }
    sw_i2c_delay();
    acknowledged = (DL_GPIO_readPins(OLED_SDA_PORT, OLED_SDA_PIN) == 0U) ?
        1U : 0U;
    scl_low();
    return (acknowledged != 0U) ? true : false;
}

static bool write_transfer(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((data == 0) || (length == 0U)) {
        return false;
    }
    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t) (s_i2c_address << 1U))) {
        sw_i2c_stop();
        g_oled_last_error = OLED_ERROR_TRANSFER;
        return false;
    }
    for (index = 0U; index < length; index++) {
        if (!sw_i2c_write_byte(data[index])) {
            sw_i2c_stop();
            g_oled_last_error = OLED_ERROR_TRANSFER;
            return false;
        }
    }
    sw_i2c_stop();
    return true;
}

static void sw_i2c_init(void)
{
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(16U);
    /* Keep input enabled while switching the output-enable bit for open drain. */
    DL_GPIO_initDigitalInputFeatures(
        OLED_SCL_IOMUX, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        OLED_SDA_IOMUX, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(OLED_SCL_PORT, OLED_SCL_PIN);
    DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN);
    DL_GPIO_disableOutput(OLED_SCL_PORT, OLED_SCL_PIN);
    DL_GPIO_disableOutput(OLED_SDA_PORT, OLED_SDA_PIN);
}

/* A warm MCU reset can occur while the SSD1306 is still powered from the
 * external 12-V supply.  In that case the display may retain an incomplete
 * I2C transaction from just before reset.  Release both lines, clock out a
 * possible partial byte, then issue a STOP before probing the display. */
static void sw_i2c_recover_bus(void)
{
    uint8_t pulse;

    sda_release();
    for (pulse = 0U; pulse < 9U; pulse++) {
        scl_low();
        sw_i2c_delay();
        (void) scl_release_and_wait_high();
        sw_i2c_delay();
    }
    sw_i2c_stop();
}

static void write_text(uint8_t startColumn, const char *text)
{
    uint8_t column = startColumn;

    while ((*text != '\0') && (column < OLED_PAGE_WIDTH)) {
        const uint8_t *glyph = glyph_for(*text++);
        uint8_t glyphColumn;

        for (glyphColumn = 0U;
             (glyphColumn < 5U) && (column < OLED_PAGE_WIDTH);
             glyphColumn++) {
            s_page_data[column++] = glyph[glyphColumn];
        }
        if (column < OLED_PAGE_WIDTH) {
            s_page_data[column++] = 0U;
        }
    }
}

static uint8_t append_unsigned(char *text, uint8_t index, uint32_t value,
                               uint8_t minimumDigits)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(digits)));
    while ((count < minimumDigits) && (count < sizeof(digits))) {
        digits[count++] = '0';
    }
    while (count != 0U) {
        text[index++] = digits[--count];
    }
    text[index] = '\0';
    return index;
}

static uint8_t append_signed_degrees(char *text, uint8_t index,
                                     int32_t millidegrees)
{
    uint32_t magnitude;

    if (millidegrees < 0) {
        text[index++] = '-';
        magnitude = (uint32_t) (-millidegrees);
    } else {
        text[index++] = '+';
        magnitude = (uint32_t) millidegrees;
    }
    index = append_unsigned(text, index, magnitude / 1000U, 1U);
    text[index++] = '.';
    index = append_unsigned(text, index, (magnitude % 1000U) / 100U, 1U);
    return index;
}

static uint8_t append_signed_integer(char *text, uint8_t index,
                                     int32_t value, uint8_t digits)
{
    if (value < 0) {
        text[index++] = '-';
        return append_unsigned(text, index, (uint32_t) (-value), digits);
    }
    text[index++] = '+';
    return append_unsigned(text, index, (uint32_t) value, digits);
}

static char hex_digit(uint8_t value)
{
    return (value < 10U) ? (char) ('0' + value) :
           (char) ('A' + (value - 10U));
}

static void make_page(uint8_t page)
{
    char text[22];
    uint8_t index = 0U;
    const int32_t headingError = s_status.headingMdeg -
                                 s_status.activeTargetMdeg;
    uint8_t column;

    for (column = 0U; column < OLED_PAGE_WIDTH; column++) {
        s_page_data[column] = 0U;
    }

    if (s_text_mode != 0U) {
        write_text(0U, s_text_lines[page]);
        return;
    }

    switch (page) {
    case 0U:
        if (s_status.functionId == 2U) {
            text[index++] = 'T'; text[index++] = 'A'; text[index++] = 'S';
            text[index++] = 'K'; text[index++] = '2'; text[index++] = ' ';
            text[index++] = 'L'; text[index++] = 'A'; text[index++] = 'P';
        } else {
            text[index++] = 'H'; text[index++] = 'D'; text[index++] = 'G';
            index = append_signed_degrees(text, index, s_status.headingMdeg);
        }
        break;
    case 1U:
        text[index++] = 'T'; text[index++] = 'G'; text[index++] = 'T';
        index = append_signed_degrees(text, index, s_status.activeTargetMdeg);
        text[index++] = 'P';
        index = append_unsigned(text, index, s_status.arcPhase, 1U);
        text[index++] = 'A';
        index = append_unsigned(text, index, s_status.arcNumber, 1U);
        break;
    case 2U:
        text[index++] = 'E'; text[index++] = 'R'; text[index++] = 'R';
        index = append_signed_degrees(text, index, headingError);
        text[index++] = 'R';
        text[index++] = (s_status.arcDirection < 0) ? '-' :
                        ((s_status.arcDirection > 0) ? '+' : '0');
        text[index++] = 'W';
        index = append_unsigned(text, index, s_status.whiteConfirmMs, 3U);
        break;
    case 3U:
        text[index++] = 'S'; text[index++] = '=';
        text[index++] = hex_digit((uint8_t) (s_status.blackMask >> 4U));
        text[index++] = hex_digit((uint8_t) (s_status.blackMask & 0x0FU));
        text[index++] = 'Q';
        index = append_unsigned(text, index, s_status.lineConfirmMs, 2U);
        text[index++] = 'E';
        index = append_signed_integer(text, index, s_status.lineError, 3U);
        text[index++] = 'F';
        index = append_unsigned(text, index, s_status.fixedProfile, 1U);
        break;
    case 4U:
        text[index++] = 'P'; text[index++] = 'W'; text[index++] = 'M';
        index = append_unsigned(text, index, s_status.leftPwm, 2U);
        text[index++] = '/';
        index = append_unsigned(text, index, s_status.rightPwm, 2U);
        text[index++] = 'C';
        index = append_unsigned(text, index, s_status.driveCommand, 1U);
        break;
    case 5U:
        text[index++] = 'I'; text[index++] = 'M'; text[index++] = 'U';
        text[index++] = s_status.imuHealthy != 0U ? '1' : '0';
        text[index++] = 'W';
        text[index++] = hex_digit((uint8_t) (s_status.imuWhoAmI >> 4U));
        text[index++] = hex_digit((uint8_t) (s_status.imuWhoAmI & 0x0FU));
        text[index++] = 'E';
        index = append_unsigned(text, index, s_status.imuLastError, 1U);
        text[index++] = 'F';
        index = append_unsigned(text, index, s_status.imuFailures, 2U);
        text[index++] = 'D';
        index = append_unsigned(text, index,
            s_status.gyroSampleCycles / 32000U, 2U);
        break;
    case 6U:
        text[index++] = 'T'; text[index++] = 'I'; text[index++] = 'M';
        text[index++] = 'E'; text[index++] = ' ';
        index = append_unsigned(text, index, s_status.runElapsedMs / 1000U,
                                1U);
        text[index++] = '.';
        index = append_unsigned(text, index, s_status.runElapsedMs % 1000U,
                                3U);
        text[index++] = 'S';
        break;
    default:
        if (((g_oled_refresh_count / OLED_PAGE_COUNT) & 1U) == 0U) {
            text[index++] = 'G';
            index = append_signed_integer(text, index, s_status.gyroZRaw, 3U);
            text[index++] = 'C';
            index = append_signed_integer(text, index, s_status.gyroZCorrected,
                                          3U);
            text[index++] = 'B';
            index = append_signed_integer(text, index, s_status.gyroBias, 3U);
        } else {
            text[index++] = 'E'; text[index++] = 'N'; text[index++] = 'C';
            index = append_signed_integer(text, index, s_status.leftEncoder, 3U);
            text[index++] = '/';
            index = append_signed_integer(text, index, s_status.rightEncoder, 3U);
        }
        break;
    }
    text[index] = '\0';
    write_text(0U, text);
}

static bool write_page_chunk(uint8_t page, uint8_t chunk)
{
    uint8_t commands[4];
    uint8_t packet[OLED_REFRESH_COLUMNS + 1U];
    uint8_t column;
    const uint8_t startColumn = (uint8_t) (chunk * OLED_REFRESH_COLUMNS);

    commands[0] = 0x00U;             /* command stream */
    commands[1] = (uint8_t) (0xB0U | page);
    commands[2] = (uint8_t) (startColumn & 0x0FU);
    commands[3] = (uint8_t) (0x10U | (startColumn >> 4U));
    if (!write_transfer(commands, sizeof(commands))) {
        return false;
    }

    packet[0] = 0x40U;               /* data stream */
    for (column = 0U; column < OLED_REFRESH_COLUMNS; column++) {
        packet[column + 1U] = s_page_data[startColumn + column];
    }
    return write_transfer(packet, sizeof(packet));
}

bool Oled_init(void)
{
    static const uint8_t initSequence[] = {
        0x00U, 0xAEU,       /* control, display off */
        0xD5U, 0x80U,       /* clock */
        0xA8U, (OLED_DISPLAY_HEIGHT - 1U),
        0xD3U, 0x00U,       /* no display offset */
        0x40U,              /* start line */
        0x8DU, 0x14U,       /* charge pump on */
        0x20U, 0x02U,       /* page addressing */
        0xA1U, 0xC8U,       /* normal landscape orientation */
        0xDAU, (OLED_DISPLAY_HEIGHT == 64U) ? 0x12U : 0x02U,
        0x81U, 0x7FU,       /* contrast */
        0xD9U, 0xF1U, 0xDBU, 0x40U,
        0xA4U, 0xA6U, 0x2EU, 0xAFU  /* normal display, on */
    };

    g_oled_ready = 0U;
    g_oled_i2c_address = 0U;
    g_oled_last_error = OLED_ERROR_NONE;
    g_oled_last_page = 0U;
    g_oled_refresh_count = 0U;
    s_next_page = 0U;
    s_next_chunk = 0U;
    s_refresh_elapsed_ms = 0U;
    s_text_mode = 0U;
    s_i2c_address = OLED_I2C_ADDRESS_DEFAULT;
    sw_i2c_init();
    sw_i2c_recover_bus();

    if (!write_transfer(initSequence, sizeof(initSequence))) {
        s_i2c_address = OLED_I2C_ADDRESS_ALTERNATE;
        g_oled_last_error = OLED_ERROR_NONE;
        if (!write_transfer(initSequence, sizeof(initSequence))) {
            g_oled_last_error = OLED_ERROR_NOT_FOUND;
            return false;
        }
    }

    g_oled_i2c_address = s_i2c_address;
    g_oled_ready = 1U;
    return true;
}

static void refresh_display(uint16_t elapsedMs)
{
    if (g_oled_ready == 0U) {
        return;
    }
    if (s_refresh_elapsed_ms <=
        (uint16_t) (65535U - elapsedMs)) {
        s_refresh_elapsed_ms += elapsedMs;
    }
    if (s_refresh_elapsed_ms < OLED_REFRESH_INTERVAL_MS) {
        return;
    }
    s_refresh_elapsed_ms = 0U;
    if (s_next_chunk == 0U) {
        make_page(s_next_page);
    }
    if (!write_page_chunk(s_next_page, s_next_chunk)) {
        g_oled_ready = 0U;
        return;
    }
    g_oled_last_page = s_next_page;
    s_next_chunk++;
    if (s_next_chunk >= OLED_REFRESH_CHUNKS) {
        s_next_chunk = 0U;
        g_oled_refresh_count++;
        s_next_page++;
        if (s_next_page >= OLED_PAGE_COUNT) {
            s_next_page = 0U;
        }
    }
}

void Oled_updateStatus(const OledStatus *status, uint16_t elapsedMs)
{
    if (status != 0) {
        s_status = *status;
    }
    s_text_mode = 0U;
    refresh_display(elapsedMs);
}

void Oled_updateTextLines(
    const char lines[OLED_TEXT_LINE_COUNT][OLED_TEXT_LINE_MAX_CHARS + 1U],
    uint16_t elapsedMs)
{
    uint8_t line;
    uint8_t character;

    if (lines != 0) {
        for (line = 0U; line < OLED_TEXT_LINE_COUNT; line++) {
            for (character = 0U;
                 character < OLED_TEXT_LINE_MAX_CHARS;
                 character++) {
                const char value = lines[line][character];

                s_text_lines[line][character] = value;
                if (value == '\0') {
                    break;
                }
            }
            s_text_lines[line][character] = '\0';
        }
    }
    s_text_mode = 1U;
    refresh_display(elapsedMs);
}
