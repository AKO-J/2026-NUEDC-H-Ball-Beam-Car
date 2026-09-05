"""Fast K230 camera preview for pipe ROI alignment.

Upload this file together with ``ball_roi_config.py`` to the K230, open it in
CanMV IDE, and run it.  It deliberately starts no YOLO model, Wi-Fi, LCD, or
UART link, so the IDE preview is the only significant workload.

The cyan rectangle is the ROI used by deployment.  The red line is the
controller's calibrated 0 cm position (x=142).  If the pipe is not enclosed
correctly, stop the script and edit PIPE_ROI_X/Y/W/H in ball_roi_config.py.
Coordinates are in the already mirrored/flipped 320x180 image.
"""

import gc
import os
import time

from media.sensor import *

try:
    from media.media import MediaManager
except ImportError:
    MediaManager = None

try:
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
except ImportError:
    # Keeps this diagnostic script usable when it is uploaded by itself.
    FULL_FRAME_WIDTH = 320
    FULL_FRAME_HEIGHT = 180
    PIPE_ROI_X = 0
    PIPE_ROI_Y = 50
    PIPE_ROI_W = 288
    PIPE_ROI_H = 68
    SENSOR_HMIRROR = True
    SENSOR_VFLIP = True


CAMERA_ID = 2
SENSOR_FPS = 120
# This is the X=0 cm reference used by steel_ball_uart_yolo.py.
CONTROL_CENTER_X = 149

ROI_COLOR = (0, 220, 255)
CENTER_COLOR = (255, 0, 0)
TEXT_COLOR = (255, 255, 0)

try:
    IDE_EXITPOINT = os.exitpoint
except AttributeError:
    IDE_EXITPOINT = None


def poll_ide_stop():
    """Let CanMV IDE's Stop button interrupt a tight preview loop."""
    if IDE_EXITPOINT:
        IDE_EXITPOINT()


def create_sensor():
    try:
        sensor = Sensor(
            id=CAMERA_ID,
            width=FULL_FRAME_WIDTH,
            height=FULL_FRAME_HEIGHT,
            fps=SENSOR_FPS,
        )
    except TypeError:
        # Older firmware has no ``fps`` constructor argument.
        sensor = Sensor(
            id=CAMERA_ID,
            width=FULL_FRAME_WIDTH,
            height=FULL_FRAME_HEIGHT,
        )
    sensor.reset()
    sensor.set_framesize(
        width=FULL_FRAME_WIDTH,
        height=FULL_FRAME_HEIGHT,
        chn=CAM_CHN_ID_1,
    )
    # RGB565 is sufficient for a fast IDE preview and costs less bandwidth
    # than the RGB888 KPU input used by the full detection application.
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
    sensor.set_hmirror(SENSOR_HMIRROR)
    sensor.set_vflip(SENSOR_VFLIP)
    if MediaManager:
        MediaManager.init()
    sensor.run()
    return sensor


def draw_guides(img, fps):
    """Draw deployment geometry without masking any part of the live image."""
    img.draw_rectangle(
        PIPE_ROI_X, PIPE_ROI_Y, PIPE_ROI_W, PIPE_ROI_H,
        color=ROI_COLOR, thickness=2,
    )
    img.draw_line(
        CONTROL_CENTER_X, 0, CONTROL_CENTER_X, FULL_FRAME_HEIGHT - 1,
        color=CENTER_COLOR, thickness=1,
    )
    img.draw_string_advanced(
        4, 4, 14,
        "ROI %d,%d %dx%d" % (PIPE_ROI_X, PIPE_ROI_Y, PIPE_ROI_W, PIPE_ROI_H),
        color=TEXT_COLOR,
    )
    img.draw_string_advanced(
        4, 22, 14,
        "X0=%d  %.1f FPS" % (CONTROL_CENTER_X, fps),
        color=TEXT_COLOR,
    )


def main():
    sensor = None
    frame_count = 0
    window_start = time.ticks_ms()
    measured_fps = 0.0
    print("PREVIEW: 320x180, camera", CAMERA_ID)
    print("ROI: %d,%d %dx%d" % (PIPE_ROI_X, PIPE_ROI_Y, PIPE_ROI_W, PIPE_ROI_H))
    print("GUIDE: cyan=ROI, red=X0=%d; stop in CanMV IDE to exit" % CONTROL_CENTER_X)
    try:
        sensor = create_sensor()
        while True:
            poll_ide_stop()
            img = sensor.snapshot(chn=CAM_CHN_ID_1)
            frame_count += 1
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, window_start)
            if elapsed >= 1000:
                measured_fps = frame_count * 1000.0 / elapsed
                frame_count = 0
                window_start = now

            draw_guides(img, measured_fps)
            # CanMV IDE receives the frame from this call.
            img.compress_for_ide()
            del img
    finally:
        if sensor:
            try:
                sensor.stop()
            except Exception:
                pass
        if MediaManager:
            try:
                MediaManager.deinit()
            except Exception:
                pass
        gc.collect()


main()
