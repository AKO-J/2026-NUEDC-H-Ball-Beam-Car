"""K230 IO5/UART2_TX bench test; no camera or model is required.

Run this first with the X42S controller DISARMED. The MSPM0 OLED should show
UART3 RX OK and XOFF cycling through -40, 0, +40. Only after this link test
passes should the full steel_ball_uart_yolo.py program be used.
"""

import time

from machine import FPIOA, UART

UART_TX_IO = 5          # K230 40-pin header pin 17
UART_BAUD = 921600
TEST_X_OFFSETS = (-40, 0, 40, 0)


def send_ball(uart, frame_no, x_offset):
    frame = "B,%d,%d,%d,900,0\r\n" % (
        frame_no, time.ticks_ms(), x_offset)
    uart.write(frame.encode())
    print(frame.strip())


fpioa = FPIOA()
fpioa.set_function(UART_TX_IO, FPIOA.UART2_TXD)
uart = UART(
    UART.UART2,
    baudrate=UART_BAUD,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
)

frame_no = 0
index = 0
try:
    while True:
        send_ball(uart, frame_no, TEST_X_OFFSETS[index])
        frame_no += 1
        index = (index + 1) % len(TEST_X_OFFSETS)
        time.sleep_ms(500)
finally:
    uart.deinit()
    fpioa = None
