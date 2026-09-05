#ifndef VISION_BALL_PROTOCOL_H
#define VISION_BALL_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * K230 -> MSPM0 ASCII frame (the sole K230 UART payload in diagnostic mode):
 *   B,<frame>,<k230_ms>,<x_offset_px>,<confidence_milli>,<lost>\r\n
 *
 * ``lost`` is 1 when no qualified ball detection is available.  The image
 * width is 320 pixels, image centre is x=160, and x_offset_px is positive to
 * image right.  The K230 and MSPM0 clocks are independent; k230_ms is logged
 * for traceability, while data age is measured on the MSPM0 from RX time.
 */
enum {
    VISION_BALL_FRAME_MAX = 64,
};

typedef struct {
    uint32_t frame;
    uint32_t k230Ms;
    int16_t xOffsetPx;
    uint16_t confidenceMilli;
    uint8_t lost;
} VisionBallMeasurement;

typedef struct {
    char buffer[VISION_BALL_FRAME_MAX];
    uint8_t length;
    uint8_t collecting;
    uint32_t acceptedFrames;
    uint32_t rejectedFrames;
} VisionBallParser;

void VisionBallParser_init(VisionBallParser *parser);

/*
 * Feed one received byte. Returns true only when a complete, range-valid B
 * frame has been decoded into "measurement".
 */
bool VisionBallParser_feed(
    VisionBallParser *parser,
    uint8_t byte,
    VisionBallMeasurement *measurement);

#endif
