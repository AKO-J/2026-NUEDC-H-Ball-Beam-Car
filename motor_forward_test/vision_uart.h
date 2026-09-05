#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "vision_ball_protocol.h"

/*
 * One-way camera link:
 *   K230 header pin 17, IO5/UART2_TX -> MSPM0 pin 31, PA13/UART3_RX
 *   K230 GND                         -> MSPM0 GND
 *   UART format: 921600 baud, 8N1.
 *
 * PB12 must not be used for a return UART wire: it is the X42S DIR signal.
 */
void VisionUart_init(void);

/* Drain the interrupt RX ring and return the newest complete frame, if any. */
bool VisionUart_pollNewest(VisionBallMeasurement *measurement);

uint32_t VisionUart_getAcceptedFrameCount(void);
uint32_t VisionUart_getRejectedFrameCount(void);
uint32_t VisionUart_getDroppedByteCount(void);

#endif
