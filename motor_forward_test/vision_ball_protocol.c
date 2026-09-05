#include "vision_ball_protocol.h"

#include <stddef.h>

static bool parseUnsigned(
    const char **cursor,
    const char *end,
    uint32_t maxValue,
    uint32_t *value)
{
    uint32_t parsed = 0U;
    const char *p = *cursor;

    if ((p >= end) || (*p < '0') || (*p > '9')) {
        return false;
    }
    while ((p < end) && (*p >= '0') && (*p <= '9')) {
        const uint32_t digit = (uint32_t) (*p - '0');
        if ((digit > maxValue) ||
            (parsed > (maxValue - digit) / 10U)) {
            return false;
        }
        parsed = parsed * 10U + digit;
        ++p;
    }
    *cursor = p;
    *value = parsed;
    return true;
}

static bool parseSigned16(
    const char **cursor,
    const char *end,
    int16_t *value)
{
    bool negative = false;
    uint32_t magnitude;
    const char *p = *cursor;

    if ((p < end) && ((*p == '-') || (*p == '+'))) {
        negative = (*p == '-');
        ++p;
    }
    if (!parseUnsigned(&p, end, negative ? 32768U : 32767U, &magnitude)) {
        return false;
    }
    *cursor = p;
    *value = negative ? (int16_t) -(int32_t) magnitude :
        (int16_t) magnitude;
    return true;
}

static bool expectComma(const char **cursor, const char *end)
{
    if ((*cursor >= end) || (**cursor != ',')) {
        return false;
    }
    ++(*cursor);
    return true;
}

static bool decode(
    const char *buffer,
    uint8_t length,
    VisionBallMeasurement *measurement)
{
    const char *cursor;
    const char *end = buffer + length;
    uint32_t frame;
    uint32_t k230Ms;
    uint32_t confidence;
    uint32_t lost;
    int16_t xOffset;

    if ((length < 12U) || (measurement == NULL)) {
        return false;
    }
    cursor = buffer;
    if ((cursor[0] != 'B') || (cursor[1] != ',')) {
        return false;
    }
    cursor += 2;
    if (!parseUnsigned(&cursor, end, UINT32_MAX, &frame) ||
        !expectComma(&cursor, end) ||
        !parseUnsigned(&cursor, end, UINT32_MAX, &k230Ms) ||
        !expectComma(&cursor, end) ||
        !parseSigned16(&cursor, end, &xOffset) ||
        !expectComma(&cursor, end) ||
        !parseUnsigned(&cursor, end, 1000U, &confidence) ||
        !expectComma(&cursor, end) ||
        !parseUnsigned(&cursor, end, 1U, &lost) ||
        (cursor != end)) {
        return false;
    }

    measurement->frame = frame;
    measurement->k230Ms = k230Ms;
    measurement->xOffsetPx = xOffset;
    measurement->confidenceMilli = (uint16_t) confidence;
    measurement->lost = (uint8_t) lost;
    return true;
}

void VisionBallParser_init(VisionBallParser *parser)
{
    if (parser == NULL) {
        return;
    }
    parser->length = 0U;
    parser->collecting = 0U;
    parser->acceptedFrames = 0U;
    parser->rejectedFrames = 0U;
}

bool VisionBallParser_feed(
    VisionBallParser *parser,
    uint8_t byte,
    VisionBallMeasurement *measurement)
{
    bool decoded;

    if ((parser == NULL) || (measurement == NULL)) {
        return false;
    }
    if (byte == 'B') {
        parser->buffer[0] = 'B';
        parser->length = 1U;
        parser->collecting = 1U;
        return false;
    }
    if (parser->collecting == 0U) {
        return false;
    }
    if ((byte == '\r') || (byte == '\n')) {
        if (parser->length == 0U) {
            return false;
        }
        decoded = decode(parser->buffer, parser->length, measurement);
        parser->collecting = 0U;
        parser->length = 0U;
        if (decoded) {
            ++parser->acceptedFrames;
            return true;
        }
        ++parser->rejectedFrames;
        return false;
    }
    if (parser->length >= VISION_BALL_FRAME_MAX) {
        parser->collecting = 0U;
        parser->length = 0U;
        ++parser->rejectedFrames;
        return false;
    }
    parser->buffer[parser->length++] = (char) byte;
    return false;
}
