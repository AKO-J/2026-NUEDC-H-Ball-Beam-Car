"""CanMV K230: steel-ball detection plus a wired UART control output.

Upload this file to the K230 with the YOLO model available at KMODEL_PATH.
A dedicated RGBP888 channel feeds the KPU while a smaller RGB888 channel is
used for the IDE/wireless preview. The safety-critical BALL measurement is
sent independently over IO5/UART2_TX, so Wi-Fi failure never stops control.

Set ENABLE_LCD_DEBUG to True to show the CSI2 camera and an OSD diagnostic
overlay on the board's 800x480 ST7701 LCD.  This mode is intended to verify
that the box follows the physical ball before the X42S is armed.
"""

import gc
import os
import socket
import time

from libs.YOLO import YOLO11
from machine import FPIOA, UART
from media.display import Display
from media.sensor import *
import image
from ball_roi_config import (
    FULL_FRAME_HEIGHT,
    FULL_FRAME_WIDTH,
    PIPE_ROI_H,
    PIPE_ROI_W,
    PIPE_ROI_X,
    PIPE_ROI_Y,
    SENSOR_HMIRROR,
    SENSOR_VFLIP,
)

try:
    from media.media import MediaManager
except ImportError:
    MediaManager = None

try:
    IDE_EXITPOINT = os.exitpoint
except AttributeError:
    IDE_EXITPOINT = None


# Wi-Fi and receiver settings. Change these for the local network.
ENABLE_WIFI = False
# Diagnostic closed-loop mode is deliberately headless: it uses no phone or
# network video path.  The control UART below is the only telemetry output.
# Debug default: CanMV IDE on the USB-connected computer receives the
# annotated preview.  This is display-only; it neither accepts commands nor
# changes the one-way K230 -> MSPM0 control UART.
ENABLE_IDE_PREVIEW = True
# The supplied K230 3.5-inch LCD example uses an 800x480 ST7701 panel.
# LCD diagnostic mode adds a full-resolution camera channel and therefore is
# for visual verification, not maximum control-loop FPS.
ENABLE_LCD_DEBUG = False
LCD_DISPLAY_TYPE = Display.ST7701
LCD_WIDTH = 800
LCD_HEIGHT = 480
WIFI_SSID = "K230_LINK"
WIFI_PASSWORD = "CHANGE_ME"
SERVER_IP = "192.168.137.1"
SERVER_PORT = 8000
VIEWER_URL = "http://%s:%d" % (SERVER_IP, SERVER_PORT)

# Model settings. The verified deployment asset is kept at
# k230/models/yolo11n_det_320.kmodel in this project. Upload/overwrite it on
# the K230 SD card at the exact KMODEL_PATH below before running this script.
KMODEL_PATH = "/sdcard/libs/yolo11n_det_320.kmodel"
LABELS = {0: "STEEL BALL"}
MODEL_INPUT_SIZE = [320, 320]
# The AI channel first captures the complete 320x180 frame. Software then
# slices the displayed pipe ROI before YOLO, and post-processing translates
# boxes back into this full-frame coordinate system before UART.
AI_FRAME_SIZE = [PIPE_ROI_W, PIPE_ROI_H]
STREAM_FRAME_SIZE = [FULL_FRAME_WIDTH, FULL_FRAME_HEIGHT]
# Ball position calibration, 2026-08-01 (full 320 px image coordinates):
#   -5 cm: x=84, 0 cm: x=149, +5 cm: x=208.
# Keep the measured asymmetry instead of forcing one scale on both sides.
# The controller receives X OFFSET relative to this measured 0 cm point.
CONTROL_CENTER_X = 149
CONTROL_CENTER_Y = STREAM_FRAME_SIZE[1] // 2
PIXELS_PER_CM_POSITIVE = 11.8
PIXELS_PER_CM_NEGATIVE = 13.0
PLACEMENT_TOLERANCE_MM = 2.0
PLACEMENT_TOLERANCE_PX = (
    min(PIXELS_PER_CM_POSITIVE, PIXELS_PER_CM_NEGATIVE)
    * PLACEMENT_TOLERANCE_MM / 10.0
)
CONTROL_MARK_SIZE = 7
CONTROL_CENTER_COLOR = (255, 0, 0)
BALL_CENTER_COLOR = (0, 255, 0)
X_MEASURE_COLOR = (255, 255, 0)
CONFIDENCE_THRESHOLD = 0.32
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 5
HOLD_MISSED_DETECTION_RUNS = 1

# The fixed ball-track ROI is imported from ball_roi_config.py and applied to
# the mirrored/flipped full image immediately before inference.
PIPE_ROI_COLOR = (0, 180, 255)
PIPE_ROI_MASK_COLOR = (0, 0, 0)

# Camera/upload settings.
CAMERA_ID = 2
# Camera acquisition target. Capture at the same 320x180 geometry used by the
# model and IDE preview, avoiding an otherwise unnecessary sensor/ISP resize.
# This is the profile requested from the OV5647 for 120 FPS capture.
# If the installed sensor/firmware does not provide this mode, CanMV will fall
# back to its nearest supported rate; use the IDE performance log to verify.
SENSOR_MODE_WIDTH = 320
SENSOR_MODE_HEIGHT = 180
SENSOR_MODE_FPS = 120
JPEG_QUALITY = 25
TARGET_FPS = 120
# Run YOLO on every captured frame. This maximizes detection/control update
# rate, but the actual capture loop rate is now bounded by KPU inference time.
DETECT_EVERY_N_FRAMES = 1
# IDE JPEG compression is substantially more expensive than drawing or UART.
# Send one preview for every two capture frames; this targets an approximately
# 60 FPS IDE view while retaining processing time for capture and inference.
IDE_PREVIEW_EVERY_N_FRAMES = 2
PERF_WINDOW_MS = 5000
DRAW_LABELS = False
# Avoid a frequent full-GC pause during high-rate capture. Object lifetimes are
# still explicitly released after each inference frame.
GC_EVERY_N_FRAMES = 600
CONNECT_RETRY_SECONDS = 2
WIFI_CONNECT_ATTEMPTS = 3
WEBSITE_CONNECT_ATTEMPTS = 3

# One-way control UART. K230 header pin 17 is IO5/UART2_TX.
# Wire it to MSPM0 pin 31, PA13/UART3_RX, plus one common GND wire.
ENABLE_CONTROL_UART = True
CONTROL_UART_ID = UART.UART2
CONTROL_UART_TX_IO = 5
# 921600 baud keeps the short one-way K230->MSPM0 coordinate link well ahead
# of the 120 FPS capture loop. Both ends must use this same setting.
CONTROL_UART_BAUD = 921600
# Leave a short, interruptible window before high-load camera/KPU startup so
# CanMV IDE can attach and stop an auto-started /sdcard/main.py cleanly.
AUTO_START_DELAY_MS = 3000


def poll_ide_stop():
    if IDE_EXITPOINT:
        IDE_EXITPOINT()


def is_ide_stop(error):
    return isinstance(error, KeyboardInterrupt) or "IDE interrupt" in str(error)


def interruptible_sleep_ms(duration_ms):
    deadline = time.ticks_add(time.ticks_ms(), duration_ms)
    while True:
        poll_ide_stop()
        remaining = time.ticks_diff(deadline, time.ticks_ms())
        if remaining <= 0:
            return
        time.sleep_ms(min(remaining, 50))


def connect_wifi():
    import network

    wlan = network.WLAN(network.STA_IF)
    try:
        wlan.active(True)
    except Exception as error:
        if is_ide_stop(error):
            raise
        print("Wi-Fi active warning:", error)

    if not wlan.isconnected():
        for attempt in range(WIFI_CONNECT_ATTEMPTS):
            poll_ide_stop()
            print("STATUS: Connecting to Wi-Fi", WIFI_SSID, "attempt", attempt + 1)
            try:
                wlan.connect(WIFI_SSID, WIFI_PASSWORD)
            except Exception as error:
                if is_ide_stop(error):
                    raise
                print("Wi-Fi connect call failed:", error)
            deadline = time.ticks_add(time.ticks_ms(), 15000)
            while not wlan.isconnected():
                poll_ide_stop()
                if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                    break
                time.sleep_ms(250)
            if wlan.isconnected():
                break
            interruptible_sleep_ms(1000)

    if not wlan.isconnected():
        print("STATUS: Wi-Fi unavailable after", WIFI_CONNECT_ATTEMPTS, "attempts")
        return None
    print("STATUS: Wi-Fi connected", wlan.ifconfig())
    print("WEBSITE:", VIEWER_URL)
    return wlan


def image_to_ai_tensor(img, roi=None):
    """Return contiguous NCHW data, optionally cropped after full-frame capture."""
    array = img.to_numpy_ref()
    shape = array.shape
    if len(shape) == 4:
        if roi is None:
            return array
        x, y, width, height = roi
        return array[:, :, y:y + height, x:x + width].copy()
    if len(shape) == 3 and shape[0] == 3:
        if roi is not None:
            x, y, width, height = roi
            array = array[:, y:y + height, x:x + width].copy()
            shape = array.shape
        return array.reshape((1, shape[0], shape[1], shape[2]))
    if len(shape) == 3 and shape[2] == 3:
        if roi is not None:
            x, y, width, height = roi
            array = array[y:y + height, x:x + width, :].copy()
            shape = array.shape
        height, width, channels = shape
        flat = array.reshape((height * width, channels))
        return flat.transpose().copy().reshape((1, channels, height, width))
    raise RuntimeError("unexpected AI channel shape: %s" % (shape,))


def image_to_jpeg_bytes(img):
    if hasattr(img, "to_jpeg"):
        encoded = img.to_jpeg(quality=JPEG_QUALITY)
    elif hasattr(img, "compress"):
        encoded = img.compress(quality=JPEG_QUALITY)
        if encoded is None:
            encoded = img
    else:
        raise RuntimeError("camera image has no JPEG encoder")

    for name in ("to_bytes", "bytes", "bytearray"):
        if hasattr(encoded, name):
            value = getattr(encoded, name)
            return value() if callable(value) else value
    return bytes(encoded)


def detection_is_in_pipe_roi(x, y, width, height):
    """Accept a box only when its centre lies inside the physical ball track."""
    center_x = x + width // 2
    center_y = y + height // 2
    return (
        PIPE_ROI_X <= center_x < PIPE_ROI_X + PIPE_ROI_W
        and PIPE_ROI_Y <= center_y < PIPE_ROI_Y + PIPE_ROI_H
    )


def mask_outside_pipe_roi(img):
    """Show only the pipe area in IDE/MJPEG without changing X coordinates."""
    frame_w, frame_h = STREAM_FRAME_SIZE
    roi_right = PIPE_ROI_X + PIPE_ROI_W
    roi_bottom = PIPE_ROI_Y + PIPE_ROI_H

    if PIPE_ROI_Y > 0:
        img.draw_rectangle(
            0, 0, frame_w, PIPE_ROI_Y,
            color=PIPE_ROI_MASK_COLOR, fill=True,
        )
    if roi_bottom < frame_h:
        img.draw_rectangle(
            0, roi_bottom, frame_w, frame_h - roi_bottom,
            color=PIPE_ROI_MASK_COLOR, fill=True,
        )
    if PIPE_ROI_X > 0:
        img.draw_rectangle(
            0, PIPE_ROI_Y, PIPE_ROI_X, PIPE_ROI_H,
            color=PIPE_ROI_MASK_COLOR, fill=True,
        )
    if roi_right < frame_w:
        img.draw_rectangle(
            roi_right, PIPE_ROI_Y, frame_w - roi_right, PIPE_ROI_H,
            color=PIPE_ROI_MASK_COLOR, fill=True,
        )
    img.draw_rectangle(
        PIPE_ROI_X, PIPE_ROI_Y, PIPE_ROI_W, PIPE_ROI_H,
        color=PIPE_ROI_COLOR, thickness=1,
    )


def cache_detection_result(result):
    """Translate crop-local detector boxes into full-frame coordinates."""
    detections = []
    if not result:
        return detections
    boxes, class_ids, scores = result[0], result[1], result[2]
    for index in range(len(boxes)):
        x, y, width, height = map(lambda value: int(round(value, 0)), boxes[index])
        class_id = int(class_ids[index])
        # The retrained deployment model is single-class: steel ball only.
        # This prevents a wrong multi-class kmodel from steering the beam.
        if class_id != 0:
            continue
        x += PIPE_ROI_X
        y += PIPE_ROI_Y
        if detection_is_in_pipe_roi(x, y, width, height):
            detections.append((x, y, width, height, class_id, float(scores[index])))
    return detections


def draw_cached_detections(detections, img, yolo):
    """Draw only the single highest-confidence target in the pipe ROI."""
    detection = primary_detection(detections)
    if detection is None:
        return
    x, y, width, height, class_id, score = detection
    color = yolo.colors[class_id]
    img.draw_rectangle(x, y, width, height, color=color, thickness=4)
    if DRAW_LABELS:
        text_y = y - 28 if y >= 28 else y
        text = "%s %.2f" % (LABELS[class_id], score)
        img.draw_string_advanced(x, text_y, 20, text, color=color)


def primary_detection(detections):
    best = None
    for detection in detections:
        if best is None or detection[5] > best[5]:
            best = detection
    return best


def control_measurement(detections):
    """Return the highest-confidence pipe-ROI target without drawing it."""
    detection = primary_detection(detections)
    if detection is None:
        return None, 0
    x, _, width, _, _, score = detection
    x_offset = x + width // 2 - CONTROL_CENTER_X
    confidence_milli = max(0, min(1000, int(round(score * 1000))))
    return x_offset, confidence_milli


def offset_px_to_mm(x_offset_px):
    """Convert signed camera offset into beam distance using the recorded scale."""
    pixels_per_cm = (PIXELS_PER_CM_POSITIVE if x_offset_px >= 0
                     else PIXELS_PER_CM_NEGATIVE)
    return x_offset_px * 10.0 / pixels_per_cm


def placement_is_ready(x_offset_px):
    return (x_offset_px is not None and
            abs(x_offset_px) <= PLACEMENT_TOLERANCE_PX)


def draw_control_overlay(detections, img):
    center_x = CONTROL_CENTER_X
    center_y = CONTROL_CENTER_Y
    mark = CONTROL_MARK_SIZE
    img.draw_line(
        center_x - mark,
        center_y,
        center_x + mark,
        center_y,
        color=CONTROL_CENTER_COLOR,
        thickness=2,
    )
    img.draw_line(
        center_x,
        center_y - mark,
        center_x,
        center_y + mark,
        color=CONTROL_CENTER_COLOR,
        thickness=2,
    )

    detection = primary_detection(detections)
    if detection is None:
        img.draw_string_advanced(6, 6, 16, "X OFFSET: --", color=X_MEASURE_COLOR)
        return None, 0

    x, y, width, height, _, score = detection
    ball_x = x + width // 2
    ball_y = y + height // 2
    x_offset = ball_x - center_x

    # The horizontal segment is the signed X distance used by the controller.
    img.draw_line(
        center_x,
        center_y,
        ball_x,
        center_y,
        color=X_MEASURE_COLOR,
        thickness=2,
    )
    img.draw_line(
        ball_x,
        center_y,
        ball_x,
        ball_y,
        color=X_MEASURE_COLOR,
        thickness=1,
    )
    img.draw_line(
        ball_x - 4,
        ball_y,
        ball_x + 4,
        ball_y,
        color=BALL_CENTER_COLOR,
        thickness=2,
    )
    img.draw_line(
        ball_x,
        ball_y - 4,
        ball_x,
        ball_y + 4,
        color=BALL_CENTER_COLOR,
        thickness=2,
    )
    placement_mm = offset_px_to_mm(x_offset)
    placement = "READY" if placement_is_ready(x_offset) else "MOVE BALL"
    img.draw_string_advanced(
        6,
        6,
        16,
        "X:%d  dX:%+dpx (%+.1fmm) %s" % (
            ball_x, x_offset, placement_mm, placement
        ),
        color=X_MEASURE_COLOR,
    )
    confidence_milli = max(0, min(1000, int(round(score * 1000))))
    return x_offset, confidence_milli


def create_control_uart():
    """Configure only K230 TX; there is no MSPM0->K230 return wire."""
    if not ENABLE_CONTROL_UART:
        return None, None
    fpioa = FPIOA()
    fpioa.set_function(CONTROL_UART_TX_IO, FPIOA.UART2_TXD)
    uart = UART(
        CONTROL_UART_ID,
        baudrate=CONTROL_UART_BAUD,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    print(
        "STATUS: Control UART ready IO%d TX %d 8N1"
        % (CONTROL_UART_TX_IO, CONTROL_UART_BAUD)
    )
    return fpioa, uart


def send_ball_measurement(
        uart, frame, k230_ms, x_offset_px, confidence_milli, lost):
    """Send the closed-loop diagnostic record consumed by the MSPM0.

    This is intentionally the only K230-to-TI UART payload.  ``lost`` is 1
    when no qualified ball detection is available, otherwise 0.
    """
    if uart is None:
        return
    uart.write(("B,%d,%d,%d,%d,%d\r\n" % (
        frame,
        k230_ms,
        x_offset_px,
        confidence_milli,
        1 if lost else 0,
    )).encode())


def send_all(sock, data):
    view = memoryview(data)
    offset = 0
    while offset < len(view):
        sent = sock.send(view[offset:])
        if sent is None:
            sent = len(view) - offset
        if sent <= 0:
            raise OSError("socket send failed")
        offset += sent


def open_uplink():
    gc.collect()
    sock = None
    try:
        sock = socket.socket()
        try:
            # Avoid delayed-ACK stalls between the small HTTP headers and JPEG.
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
        sock.settimeout(5)
        sock.connect((SERVER_IP, SERVER_PORT))
        sock.settimeout(0.25)
        request = (
            "POST /api/mjpeg HTTP/1.1\r\n"
            "Host: %s:%d\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "Transfer-Encoding: chunked\r\n"
            "Connection: keep-alive\r\n\r\n"
        ) % (SERVER_IP, SERVER_PORT)
        send_all(sock, request.encode())
        print("STATUS: Website receiver connected")
        print("WEBSITE:", VIEWER_URL)
        return sock
    except Exception:
        if sock:
            try:
                sock.close()
            except Exception:
                pass
        gc.collect()
        raise


def open_uplink_with_retries():
    for attempt in range(WEBSITE_CONNECT_ATTEMPTS):
        poll_ide_stop()
        print(
            "STATUS: Connecting to website receiver attempt",
            attempt + 1,
        )
        try:
            return open_uplink()
        except OSError as error:
            print("STATUS: Website receiver unavailable", error)
            if attempt + 1 < WEBSITE_CONNECT_ATTEMPTS:
                interruptible_sleep_ms(CONNECT_RETRY_SECONDS * 1000)
    print("STATUS: Offline IDE mode; wireless video disabled")
    return None


def send_frame(sock, jpeg):
    # Keep the large JPEG separate while coalescing the two small wire headers.
    part_header = (
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
        % len(jpeg)
    ).encode()
    payload_length = len(part_header) + len(jpeg) + 2
    # Three writes instead of five substantially reduces lwIP/Nagle overhead.
    wire_header = ("%X\r\n" % payload_length).encode() + part_header
    send_all(sock, wire_header)
    send_all(sock, jpeg)
    send_all(sock, b"\r\n\r\n")


def create_lcd():
    """Initialise the on-board 3.5-inch ST7701 LCD and one OSD layer."""
    if not ENABLE_LCD_DEBUG:
        return None
    Display.init(
        LCD_DISPLAY_TYPE,
        width=LCD_WIDTH,
        height=LCD_HEIGHT,
        osd_num=1,
        to_ide=False,
    )
    print("STATUS: LCD ST7701 %dx%d ready" % (LCD_WIDTH, LCD_HEIGHT))
    return image.Image(LCD_WIDTH, LCD_HEIGHT, image.ARGB8888)


def create_sensor():
    capture_width = LCD_WIDTH if ENABLE_LCD_DEBUG else SENSOR_MODE_WIDTH
    capture_height = LCD_HEIGHT if ENABLE_LCD_DEBUG else SENSOR_MODE_HEIGHT
    try:
        sensor = Sensor(
            id=CAMERA_ID,
            width=capture_width,
            height=capture_height,
            fps=SENSOR_MODE_FPS,
        )
    except TypeError:
        # Compatibility with firmware versions whose constructor has no fps.
        sensor = Sensor(
            id=CAMERA_ID,
            width=capture_width,
            height=capture_height,
        )
    sensor.reset()
    if ENABLE_LCD_DEBUG:
        # Channel 0 is routed directly to the LCD video layer.  Channels 1
        # and 2 below remain dedicated to optional IDE preview and YOLO.
        sensor.set_framesize(width=LCD_WIDTH, height=LCD_HEIGHT)
        sensor.set_pixformat(Sensor.YUV420SP)
        bind_info = sensor.bind_info(x=0, y=0)
        Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)
    sensor.set_framesize(
        width=STREAM_FRAME_SIZE[0],
        height=STREAM_FRAME_SIZE[1],
        chn=CAM_CHN_ID_1,
    )
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_1)
    sensor.set_framesize(
        width=FULL_FRAME_WIDTH,
        height=FULL_FRAME_HEIGHT,
        chn=CAM_CHN_ID_2,
    )
    sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
    sensor.set_hmirror(SENSOR_HMIRROR)
    sensor.set_vflip(SENSOR_VFLIP)
    if MediaManager:
        MediaManager.init()
    sensor.run()
    print("STATUS: Camera ready, requested %dx%d @ %d FPS; post-crop ROI %d,%d %dx%d" % (
        capture_width, capture_height, SENSOR_MODE_FPS,
        PIPE_ROI_X, PIPE_ROI_Y, PIPE_ROI_W, PIPE_ROI_H))
    return sensor


def snapshot_stream(sensor):
    return sensor.snapshot(chn=CAM_CHN_ID_1)


def snapshot_ai(sensor):
    return sensor.snapshot(chn=CAM_CHN_ID_2)


def draw_lcd_overlay(osd_img, detections, x_offset_px, confidence_milli):
    """Draw scaled ROI plus calibrated millimetre placement guidance on LCD."""
    scale_x = LCD_WIDTH / STREAM_FRAME_SIZE[0]
    scale_y = LCD_HEIGHT / STREAM_FRAME_SIZE[1]
    osd_img.clear()

    # Cyan: the only region from which a control target may be selected.
    osd_img.draw_rectangle(
        int(PIPE_ROI_X * scale_x), int(PIPE_ROI_Y * scale_y),
        int(PIPE_ROI_W * scale_x), int(PIPE_ROI_H * scale_y),
        color=(0, 180, 255, 255), thickness=2,
    )
    center_x = int(CONTROL_CENTER_X * scale_x)
    osd_img.draw_line(center_x, 0, center_x, LCD_HEIGHT,
                      color=(255, 0, 0, 255), thickness=2)
    tolerance_px = max(1, int(round(PLACEMENT_TOLERANCE_PX * scale_x)))
    osd_img.draw_line(center_x - tolerance_px, 0,
                      center_x - tolerance_px, LCD_HEIGHT,
                      color=(0, 255, 0, 255), thickness=1)
    osd_img.draw_line(center_x + tolerance_px, 0,
                      center_x + tolerance_px, LCD_HEIGHT,
                      color=(0, 255, 0, 255), thickness=1)
    osd_img.draw_string_advanced(
        12, 12, 20,
        "C=%dpx  PX/CM +%.1f/-%.1f  BAND +/-%0.1fmm" % (
            CONTROL_CENTER_X, PIXELS_PER_CM_POSITIVE,
            PIXELS_PER_CM_NEGATIVE, PLACEMENT_TOLERANCE_MM
        ),
        color=(255, 255, 0, 255),
    )

    detection = primary_detection(detections)
    if detection is None:
        osd_img.draw_string_advanced(12, 38, 24, "BALL: NONE",
                                     color=(255, 80, 80, 255))
        return

    x, y, width, height, _, score = detection
    draw_x = int(x * scale_x)
    draw_y = int(y * scale_y)
    draw_w = max(1, int(width * scale_x))
    draw_h = max(1, int(height * scale_y))
    osd_img.draw_rectangle(draw_x, draw_y, draw_w, draw_h,
                           color=(0, 255, 0, 255), thickness=3)
    placement_mm = offset_px_to_mm(x_offset_px)
    status = "READY" if placement_is_ready(x_offset_px) else "MOVE BALL"
    label = "X=%d  dX=%+dpx  %+.1fmm  %s  C%d" % (
        x + width // 2, x_offset_px, placement_mm, status,
        confidence_milli,
    )
    status_color = ((0, 255, 0, 255) if placement_is_ready(x_offset_px)
                    else (255, 180, 0, 255))
    osd_img.draw_string_advanced(draw_x, max(38, draw_y - 25), 20, label,
                                 color=status_color)


def main():
    if IDE_EXITPOINT:
        exitpoint_enable = getattr(os, "EXITPOINT_ENABLE", None)
        if exitpoint_enable is not None:
            IDE_EXITPOINT(exitpoint_enable)

    print("STATUS: main.py starts in %d ms" % AUTO_START_DELAY_MS)
    interruptible_sleep_ms(AUTO_START_DELAY_MS)

    control_fpioa, control_uart = create_control_uart()
    wlan = connect_wifi() if ENABLE_WIFI else None
    uplink = open_uplink_with_retries() if wlan else None
    if uplink is None and wlan is None:
        print("STATUS: Wireless video disabled; UART control remains active")

    lcd_osd = create_lcd()
    sensor = create_sensor()
    yolo = YOLO11(
        task_type="detect",
        mode="video",
        kmodel_path=KMODEL_PATH,
        labels=LABELS,
        rgb888p_size=AI_FRAME_SIZE,
        model_input_size=MODEL_INPUT_SIZE,
        # Keep postprocess boxes in the 288x48 AI-crop coordinate system.
        # cache_detection_result() below restores the full 320x180 origin.
        # Using STREAM_FRAME_SIZE here would stretch crop-local boxes before
        # that restoration, shifting the sent X coordinate and rejecting Y.
        display_size=AI_FRAME_SIZE,
        conf_thresh=CONFIDENCE_THRESHOLD,
        nms_thresh=NMS_THRESHOLD,
        max_boxes_num=MAX_BOXES_NUM,
        debug_mode=0,
    )
    yolo.config_preprocess()

    frame_no = 0
    detect_every_n_frames = DETECT_EVERY_N_FRAMES
    cached_detections = []
    x_offset_px = None
    confidence_milli = 0
    missed_detection_runs = 0
    interval = max(1, 1000 // TARGET_FPS)
    next_at = time.ticks_ms()
    perf_window_started = next_at
    perf_window_frames = 0
    perf_detection_runs = 0
    try:
        while True:
            poll_ide_stop()
            try:
                if frame_no % detect_every_n_frames == 0:
                    ai_img = snapshot_ai(sensor)
                    tensor = image_to_ai_tensor(
                        ai_img,
                        (PIPE_ROI_X, PIPE_ROI_Y, PIPE_ROI_W, PIPE_ROI_H),
                    )
                    result = yolo.run(tensor)
                    perf_detection_runs += 1
                    new_detections = cache_detection_result(result)
                    if new_detections:
                        cached_detections = new_detections
                        missed_detection_runs = 0
                    elif missed_detection_runs < HOLD_MISSED_DETECTION_RUNS:
                        missed_detection_runs += 1
                    else:
                        cached_detections = []
                    del tensor
                    del result
                    del new_detections
                    del ai_img
                img = None
                if uplink or ENABLE_IDE_PREVIEW:
                    img = snapshot_stream(sensor)
                    mask_outside_pipe_roi(img)
                    draw_cached_detections(cached_detections, img, yolo)
                    x_offset_px, confidence_milli = draw_control_overlay(
                        cached_detections, img
                    )
                else:
                    # Headless control path: no second camera snapshot, no
                    # drawing and no IDE JPEG compression.
                    x_offset_px, confidence_milli = control_measurement(
                        cached_detections
                    )
                send_ball_measurement(
                    control_uart,
                    frame_no,
                    time.ticks_ms(),
                    0 if x_offset_px is None else x_offset_px,
                    confidence_milli,
                    x_offset_px is None,
                )
                if lcd_osd is not None:
                    draw_lcd_overlay(
                        lcd_osd, cached_detections, x_offset_px,
                        confidence_milli,
                    )
                    Display.show_image(lcd_osd, 0, 0, Display.LAYER_OSD0)
                send_ide_preview = (
                    ENABLE_IDE_PREVIEW
                    and frame_no % IDE_PREVIEW_EVERY_N_FRAMES == 0
                )
                if uplink:
                    jpeg = image_to_jpeg_bytes(img)
                    if not jpeg.startswith(b"\xff\xd8"):
                        raise RuntimeError("camera output is not JPEG")
                    try:
                        send_frame(uplink, jpeg)
                    except OSError as error:
                        print("STATUS: Website connection lost", error)
                        try:
                            uplink.close()
                        except Exception:
                            pass
                        uplink = open_uplink_with_retries()
                        if uplink is None and send_ide_preview:
                            img.compress_for_ide()
                    del jpeg
                elif send_ide_preview:
                    img.compress_for_ide()
                frame_no += 1
                perf_window_frames += 1
                now = time.ticks_ms()
                perf_elapsed = time.ticks_diff(now, perf_window_started)
                if perf_elapsed >= PERF_WINDOW_MS:
                    capture_loop_fps = perf_window_frames * 1000.0 / perf_elapsed
                    detect_fps = perf_detection_runs * 1000.0 / perf_elapsed
                    x_status = "--" if x_offset_px is None else str(x_offset_px)
                    output_mode = "LCD" if lcd_osd is not None else (
                        "WEB" if uplink else (
                            "IDE" if ENABLE_IDE_PREVIEW else "HEADLESS"
                        )
                    )
                    print(
                        "STATUS: Performance mode %s capture_loop_fps %.1f detect_fps %.1f x_px %s"
                        % (
                            output_mode,
                            capture_loop_fps,
                            detect_fps,
                            x_status,
                        )
                    )
                    perf_window_started = now
                    perf_window_frames = 0
                    perf_detection_runs = 0
                if frame_no % GC_EVERY_N_FRAMES == 0:
                    gc.collect()
                if img is not None:
                    del img
            except KeyboardInterrupt:
                raise
            except Exception as error:
                if is_ide_stop(error):
                    raise
                print("STATUS: Camera/inference error", error)
                try:
                    del img
                except Exception:
                    pass
                gc.collect()

            next_at = time.ticks_add(next_at, interval)
            remaining = time.ticks_diff(next_at, time.ticks_ms())
            if remaining > 0:
                time.sleep_ms(remaining)
            else:
                next_at = time.ticks_ms()
    except BaseException as error:
        if is_ide_stop(error):
            print("STATUS: Stop requested")
        else:
            raise
    finally:
        if uplink:
            try:
                uplink.close()
            except Exception:
                pass
        if control_uart:
            try:
                control_uart.deinit()
            except Exception:
                pass
        # Keep the FPIOA object alive until the UART is deinitialized.
        control_fpioa = None
        try:
            yolo.deinit()
        except Exception:
            pass
        try:
            sensor.stop()
        except Exception:
            pass
        if lcd_osd is not None:
            try:
                Display.deinit()
            except Exception:
                pass
        if MediaManager:
            try:
                MediaManager.deinit()
            except Exception:
                pass
        if IDE_EXITPOINT:
            try:
                exitpoint_sleep = getattr(os, "EXITPOINT_ENABLE_SLEEP", None)
                if exitpoint_sleep is not None:
                    IDE_EXITPOINT(exitpoint_sleep)
            except BaseException:
                pass
        print("STATUS: Program stopped")

try:
    main()
except BaseException as error:
    if is_ide_stop(error):
        print("STATUS: Stop requested")
        print("STATUS: Program stopped")
    else:
        raise
