"""Single source of truth for ball-track image geometry.

Copy this file with the K230 scripts. Host training tools import the same
values from the repository, so capture, training, and deployed inference use
the identical crop.
"""

FULL_FRAME_WIDTH = 320
FULL_FRAME_HEIGHT = 180

SENSOR_HMIRROR = True
SENSOR_VFLIP = True

# Coordinates in the mirrored/flipped 320x180 camera image. The crop keeps
# the whole horizontal ball track while trimming the upper/lower background
# visible in the 2026-08-01 IDE preview. On 2026-08-01 the ROI was expanded
# by about 0.5 cm (6 px at the measured 11.8--13 px/cm scale) on every edge
# that has available frame space. The left edge is already clamped at x=0.
PIPE_ROI_X = 0
PIPE_ROI_Y = 52
PIPE_ROI_W = 294
PIPE_ROI_H = 48
