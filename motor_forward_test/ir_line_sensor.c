#include "ir_line_sensor.h"

#ifndef IR_LINE_SENSOR_HOST_ONLY
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#define IR_LINE_SENSOR_O1_PORT              GPIOB
#define IR_LINE_SENSOR_O2_PORT              GPIOA
#define IR_LINE_SENSOR_O3_PORT              GPIOA
#define IR_LINE_SENSOR_O4_PORT              GPIOA
#define IR_LINE_SENSOR_O1_PIN               DL_GPIO_PIN_18
#define IR_LINE_SENSOR_O2_PIN               DL_GPIO_PIN_24
#define IR_LINE_SENSOR_O3_PIN               DL_GPIO_PIN_17
#define IR_LINE_SENSOR_O4_PIN               DL_GPIO_PIN_12
#define IR_LINE_SENSOR_PORT_A_PINS          (IR_LINE_SENSOR_O2_PIN | \
                                             IR_LINE_SENSOR_O3_PIN | \
                                             IR_LINE_SENSOR_O4_PIN)
#define IR_LINE_SENSOR_O1_IOMUX             IOMUX_PINCM44
#define IR_LINE_SENSOR_O2_IOMUX             IOMUX_PINCM54
#define IR_LINE_SENSOR_O3_IOMUX             IOMUX_PINCM39
#define IR_LINE_SENSOR_O4_IOMUX             IOMUX_PINCM34
#define IR_LINE_SENSOR_MAJORITY_GAP_CYCLES  160U

void IrLineSensor_init(void)
{
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(16U);

    /* The LF04 board supplies the comparator-output bias. Do not add an MCU
     * pull-up until the real output stage and voltage have been measured. */
    DL_GPIO_initDigitalInputFeatures(IR_LINE_SENSOR_O1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IR_LINE_SENSOR_O2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IR_LINE_SENSOR_O3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IR_LINE_SENSOR_O4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

uint8_t IrLineSensor_readRawMask(void)
{
    const uint32_t levelsA =
        DL_GPIO_readPins(GPIOA, IR_LINE_SENSOR_PORT_A_PINS);
    const uint32_t levelsB =
        DL_GPIO_readPins(GPIOB, IR_LINE_SENSOR_O1_PIN);
    uint8_t mask = 0U;

    if ((levelsB & IR_LINE_SENSOR_O1_PIN) != 0U) {
        mask |= 0x01U;
    }
    if ((levelsA & IR_LINE_SENSOR_O2_PIN) != 0U) {
        mask |= 0x02U;
    }
    if ((levelsA & IR_LINE_SENSOR_O3_PIN) != 0U) {
        mask |= 0x04U;
    }
    if ((levelsA & IR_LINE_SENSOR_O4_PIN) != 0U) {
        mask |= 0x08U;
    }
    return mask;
}

uint8_t IrLineSensor_readStableRawMask(void)
{
    const uint8_t first = IrLineSensor_readRawMask();
    uint8_t second;
    uint8_t third;

    delay_cycles(IR_LINE_SENSOR_MAJORITY_GAP_CYCLES);
    second = IrLineSensor_readRawMask();
    delay_cycles(IR_LINE_SENSOR_MAJORITY_GAP_CYCLES);
    third = IrLineSensor_readRawMask();

    return (uint8_t) ((first & second) | (first & third) | (second & third));
}
#endif /* IR_LINE_SENSOR_HOST_ONLY */

uint8_t IrLineSensor_rawToBlackMask(uint8_t rawMask,
                                    uint8_t blackLevelHigh)
{
    rawMask &= IR_LINE_SENSOR_RAW_MASK_ALL;
    return (blackLevelHigh != 0U) ? rawMask :
        (uint8_t) ((~rawMask) & IR_LINE_SENSOR_RAW_MASK_ALL);
}

uint8_t IrLineSensor_toLeftToRightMask(uint8_t o1ToO4Mask,
                                       uint8_t o1IsLeft)
{
    uint8_t result = 0U;
    uint8_t index;

    o1ToO4Mask &= IR_LINE_SENSOR_RAW_MASK_ALL;
    if (o1IsLeft != 0U) {
        return o1ToO4Mask;
    }
    for (index = 0U; index < IR_LINE_SENSOR_CHANNEL_COUNT; index++) {
        if ((o1ToO4Mask & ((uint8_t) 1U << index)) != 0U) {
            result |= ((uint8_t) 1U <<
                       (IR_LINE_SENSOR_CHANNEL_COUNT - 1U - index));
        }
    }
    return result;
}

uint8_t IrLineSensor_rawToDhWhiteState(uint8_t rawMask,
                                       uint8_t o1IsLeft)
{
    const uint8_t leftToRight =
        IrLineSensor_toLeftToRightMask(rawMask, o1IsLeft);
    uint8_t state = 0U;
    uint8_t index;

    /* leftToRight bit 0 is DH1, while the documented state stores DH1 in
     * bit 3.  Reverse the bit order without changing the electrical level. */
    for (index = 0U; index < IR_LINE_SENSOR_CHANNEL_COUNT; index++) {
        if ((leftToRight & ((uint8_t) 1U << index)) != 0U) {
            state |= (uint8_t) (1U <<
                (IR_LINE_SENSOR_CHANNEL_COUNT - 1U - index));
        }
    }
    return state;
}

uint8_t IrLineSensor_toControllerMask(uint8_t leftToRightMask)
{
    uint8_t controllerMask = 0U;

    leftToRightMask &= IR_LINE_SENSOR_RAW_MASK_ALL;
    if ((leftToRightMask & 0x01U) != 0U) {
        controllerMask |= 0x01U; /* left outer -> X1 */
    }
    if ((leftToRightMask & 0x02U) != 0U) {
        controllerMask |= 0x08U; /* left inner -> X4 */
    }
    if ((leftToRightMask & 0x04U) != 0U) {
        controllerMask |= 0x10U; /* right inner -> X5 */
    }
    if ((leftToRightMask & 0x08U) != 0U) {
        controllerMask |= 0x80U; /* right outer -> X8 */
    }
    return controllerMask;
}
