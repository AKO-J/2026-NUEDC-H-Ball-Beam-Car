#include <assert.h>
#include <stdio.h>

#include "ir_line_sensor.h"

int main(void)
{
    assert(IrLineSensor_rawToBlackMask(0x05U, 1U) == 0x05U);
    assert(IrLineSensor_rawToBlackMask(0x05U, 0U) == 0x0AU);
    assert(IrLineSensor_toLeftToRightMask(0x09U, 1U) == 0x09U);
    assert(IrLineSensor_toLeftToRightMask(0x09U, 0U) == 0x09U);
    assert(IrLineSensor_toLeftToRightMask(0x03U, 0U) == 0x0CU);
    /* White=1 DH1..DH4 table order.  With O1 physically right, connector
     * order already maps to the MSB-first DH representation. */
    assert(IrLineSensor_rawToDhWhiteState(0x09U, 0U) == 0x09U);
    assert(IrLineSensor_rawToDhWhiteState(0x01U, 0U) == 0x01U);
    assert(IrLineSensor_rawToDhWhiteState(0x01U, 1U) == 0x08U);
    assert(IrLineSensor_toControllerMask(0x01U) == 0x01U);
    assert(IrLineSensor_toControllerMask(0x02U) == 0x08U);
    assert(IrLineSensor_toControllerMask(0x04U) == 0x10U);
    assert(IrLineSensor_toControllerMask(0x08U) == 0x80U);
    assert(IrLineSensor_toControllerMask(0x06U) == 0x18U);
    assert(IrLineSensor_toControllerMask(0x0FU) == 0x99U);

    puts("ir_line_sensor_mapping_test: PASS");
    return 0;
}
