#ifndef IR_LINE_SENSOR_H_
#define IR_LINE_SENSOR_H_

#include <stdint.h>

/*
 * WHEELTEC LF04 four-channel infrared line sensor.
 *
 * Fixed wiring on LP-MSPM0G3507, using the numbered 40-pin header:
 *   LF04 O1  -> PB18 (position 25)
 *   LF04 O2  -> PA24 (position 27)
 *   LF04 O3  -> PA17 (position 28)
 *   LF04 O4  -> PA12 (position 32)
 *   LF04 GND -> LaunchPad GND
 *
 * PA10/PA11 are deliberately reserved for the onboard XDS110 UART/ROM-BSL,
 * and PA18 is the BSL-invoke input.  None is used by LF04.
 *
 * Power the LF04 from 3.3 V for the first test. If the particular board only
 * works correctly from 5 V, level-shift O1..O4 before connecting them to the
 * MSPM0. Never feed an unverified 5-V output into these GPIO pins.
 *
 * Raw-mask bit order follows the connector labels, not an assumed mounting
 * direction: bit 0=O1, bit 1=O2, bit 2=O3, bit 3=O4.
 */
#define IR_LINE_SENSOR_CHANNEL_COUNT       4U
#define IR_LINE_SENSOR_RAW_MASK_ALL        0x0FU

void IrLineSensor_init(void);

/* Read all four comparator outputs in one GPIO-port sample. */
uint8_t IrLineSensor_readRawMask(void);

/* Three quick samples followed by a per-bit majority vote. */
uint8_t IrLineSensor_readStableRawMask(void);

/* Convert the electrical levels into a four-bit mask where one means black. */
uint8_t IrLineSensor_rawToBlackMask(uint8_t rawMask,
                                    uint8_t blackLevelHigh);

/* Return a physical left-to-right mask: bit 0=leftmost, bit 3=rightmost. */
uint8_t IrLineSensor_toLeftToRightMask(uint8_t o1ToO4Mask,
                                       uint8_t o1IsLeft);

/*
 * LF04's documented control table is written as DH1 DH2 DH3 DH4, from the
 * physical left probe to the physical right probe, with white=1 and black=0.
 * This helper turns the connector-order raw levels into that table order.
 * It deliberately does not infer the mounting direction: o1IsLeft is the
 * one calibration switch that must match the installed board.
 */
uint8_t IrLineSensor_rawToDhWhiteState(uint8_t rawMask,
                                       uint8_t o1IsLeft);

/*
 * Legacy-only adapter for archived eight-position experiments.  The H-task
 * production controller uses IrLineSensor_rawToDhWhiteState() instead.
 */
uint8_t IrLineSensor_toControllerMask(uint8_t leftToRightMask);

#endif /* IR_LINE_SENSOR_H_ */
