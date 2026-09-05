"""Collect a three-minute K230 ball-in-pipe training dataset.

Every saved JPEG is the deployment-matched, physically cropped 288x48 water
pipe ROI.  Annotate these images directly; do not crop them for a second time.
The save channel uses RGB565 because CanMV v1.2 cannot directly save a
Sensor.RGB888 image.
"""

import gc
import os
import time

from media.sensor import *
# Keep this capture utility runnable when it is uploaded by itself to K230.
# The deployed program also has ball_roi_config.py, but dataset collection
# should not depend on a second file being present on the board.
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
    FULL_FRAME_WIDTH = 320
    FULL_FRAME_HEIGHT = 180
    PIPE_ROI_X = 0
    PIPE_ROI_Y = 52
    PIPE_ROI_W = 288
    PIPE_ROI_H = 48
    SENSOR_HMIRROR = True
    SENSOR_VFLIP = True

try:
    from media.media import MediaManager
except ImportError:
    MediaManager = None


CAMERA_ID = 2
FRAME_WIDTH = FULL_FRAME_WIDTH
FRAME_HEIGHT = FULL_FRAME_HEIGHT
SENSOR_FPS = 120
SAVE_FPS = 3
JPEG_QUALITY = 90
PREPARE_SECONDS = 5
# The v3 run proved that displayed ROI coordinates must not be passed directly
# as sensor-channel crop coordinates. Keep v3 as diagnostic evidence and use
# v5 for the corrected full-frame-then-crop workflow. v4 has already been
# recorded and must remain untouched.
# stage filenames restart at 0000 and would otherwise overwrite earlier data.
DATASET_ROOT = "/sdcard/datasets/ball_pipe_20260731_v5"
PROBE_SECONDS = 5

# The camera channel remains 320x180. Each saved JPEG is cropped afterward,
# using coordinates in the already mirrored/flipped full image.
SAVE_ROI_COPY = False

# 6 x 25 seconds + 30 seconds no-ball = exactly 180 seconds.
CAPTURE_PLAN = (
    ("slow_left", 25, "SLOW LEFT: x=20..95, SWEEP A-B-C-B-A"),
    ("slow_middle", 25, "SLOW MID: x=95..195, SWEEP C-D-E-D-C"),
    ("slow_right", 25, "SLOW RIGHT: x=195..270, SWEEP E-F-G-F-E"),
    ("fast_left", 25, "FAST LEFT: x=20..95, PASS BOTH DIRECTIONS"),
    ("fast_middle", 25, "FAST MID: x=95..195, PASS BOTH DIRECTIONS"),
    ("fast_right", 25, "FAST RIGHT: x=195..270, PASS BOTH DIRECTIONS"),
    ("no_ball", 30, "REMOVE BALL: NEGATIVE SAMPLES"),
)


def ensure_dir(path):
    current = ""
    for part in path.split("/"):
        if not part:
            continue
        current += "/" + part
        try:
            os.stat(current)
        except OSError:
            os.mkdir(current)


def wait_ms(milliseconds):
    deadline = time.ticks_add(time.ticks_ms(), milliseconds)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        time.sleep_ms(20)


def next_frame_index(directory, stage_name):
    """Return the next safe index, so an interrupted run can resume."""
    prefix = stage_name + "_"
    next_index = 0
    try:
        names = os.listdir(directory)
    except OSError:
        return next_index
    for name in names:
        if not (name.startswith(prefix) and name.endswith(".jpg")):
            continue
        try:
            index = int(name[len(prefix):-4])
        except ValueError:
            continue
        if index >= next_index:
            next_index = index + 1
    return next_index


def create_sensor():
    try:
        sensor = Sensor(
            id=CAMERA_ID,
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            fps=SENSOR_FPS,
        )
    except TypeError:
        sensor = Sensor(
            id=CAMERA_ID,
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
        )
    sensor.reset()
    sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT,
                         chn=CAM_CHN_ID_1)
    # CanMV v1.2's image.save() supports native RGB565 but rejects the RGB888
    # frame format used by the real-time YOLO input channel.
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
    sensor.set_hmirror(SENSOR_HMIRROR)
    sensor.set_vflip(SENSOR_VFLIP)
    if MediaManager:
        MediaManager.init()
    sensor.run()
    return sensor


def show_roi_probe(sensor):
    """Save and show the post-crop ROI before the three-minute capture."""
    wait_ms(500)
    img = sensor.snapshot(chn=CAM_CHN_ID_1)
    probe_path = DATASET_ROOT + "/roi_probe.jpg"
    img.save(probe_path, roi=(PIPE_ROI_X, PIPE_ROI_Y,
                              PIPE_ROI_W, PIPE_ROI_H),
             quality=JPEG_QUALITY)
    print("ROI PROBE:", probe_path, PIPE_ROI_W, "x", PIPE_ROI_H)
    print("CHECK: the white ball track should fill most of the 288x48 preview")
    try:
        preview = img.copy((PIPE_ROI_X, PIPE_ROI_Y,
                            PIPE_ROI_W, PIPE_ROI_H))
        preview.compress_for_ide()
        del preview
    except Exception as error:
        print("CROPPED IDE PREVIEW unavailable; inspect roi_probe.jpg:", error)
    wait_ms(PROBE_SECONDS * 1000)
    del img
    gc.collect()


def capture_stage(sensor, stage_name, duration_seconds, instruction, manifest):
    raw_dir = DATASET_ROOT + "/raw/" + stage_name
    roi_dir = DATASET_ROOT + "/roi/" + stage_name
    ensure_dir(raw_dir)
    if SAVE_ROI_COPY:
        ensure_dir(roi_dir)

    target_frames = duration_seconds * SAVE_FPS
    frame_index = next_frame_index(raw_dir, stage_name)
    if frame_index >= target_frames:
        print("SKIP:", stage_name, "already has", frame_index, "frames")
        return

    print("NEXT:", instruction)
    if frame_index:
        print("RESUME:", stage_name, "from", frame_index, "of", target_frames)
    wait_ms(PREPARE_SECONDS * 1000)
    print("CAPTURE:", stage_name, "until", target_frames, "frames")
    next_save = time.ticks_ms()

    while frame_index < target_frames:
        img = sensor.snapshot(chn=CAM_CHN_ID_1)
        now = time.ticks_ms()
        if time.ticks_diff(now, next_save) >= 0:
            filename = "%s_%04d.jpg" % (stage_name, frame_index)
            raw_path = raw_dir + "/" + filename
            img.save(raw_path, roi=(PIPE_ROI_X, PIPE_ROI_Y,
                                    PIPE_ROI_W, PIPE_ROI_H),
                     quality=JPEG_QUALITY)
            manifest.write("%s,%s,%d\n" % (stage_name, raw_path, now))
            frame_index += 1
            next_save = time.ticks_add(next_save, 1000 // SAVE_FPS)
            if frame_index % 15 == 0:
                print(stage_name, "saved", frame_index)
        del img

    print("DONE:", stage_name, "has", frame_index, "frames")
    gc.collect()


def main():
    ensure_dir(DATASET_ROOT)
    ensure_dir(DATASET_ROOT + "/raw")
    if SAVE_ROI_COPY:
        ensure_dir(DATASET_ROOT + "/roi")

    print("DATASET ROOT:", DATASET_ROOT)
    print("TOTAL: 180 seconds, approximately", 180 * SAVE_FPS, "frames")
    sensor = create_sensor()
    show_roi_probe(sensor)
    manifest_path = DATASET_ROOT + "/capture_manifest.csv"
    try:
        os.stat(manifest_path)
        manifest_exists = True
    except OSError:
        manifest_exists = False
    manifest = open(manifest_path, "a" if manifest_exists else "w")
    if not manifest_exists:
        manifest.write("stage,path,ticks_ms\n")
    metadata = open(DATASET_ROOT + "/roi_metadata.txt", "w")
    metadata.write("full=%dx%d\nroi=%d,%d,%d,%d\nworkflow=full_frame_then_crop\n" % (
        FRAME_WIDTH, FRAME_HEIGHT, PIPE_ROI_X, PIPE_ROI_Y,
        PIPE_ROI_W, PIPE_ROI_H))
    try:
        for stage_name, seconds, instruction in CAPTURE_PLAN:
            capture_stage(sensor, stage_name, seconds, instruction, manifest)
    finally:
        manifest.close()
        metadata.close()
        try:
            sensor.stop()
        except Exception:
            pass
        if MediaManager:
            try:
                MediaManager.deinit()
            except Exception:
                pass

    print("COMPLETE: copy", DATASET_ROOT, "to the computer for annotation")


main()
