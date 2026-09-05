#ifndef H_TRACK_PWM_CONFIG_H_
#define H_TRACK_PWM_CONFIG_H_

/*
 * LF04 task-2 calibration constants.  The H-track application reads the
 * H_TRACK_LF04_* names below.  The older fixed-mask constants remain only so
 * archived, non-production diagnostics can still be built.
 *
 * Encoder calibration below comes from the two 2026-07-31 straight runs:
 *   run 1: 97.0 cm, left=-6173, right=-6072
 *   run 2: 96.5 cm, left=-6298, right=-6054
 * Combined scale: left=64.45 count/cm, right=62.67 count/cm.
 */

#define H_TRACK_LF04_CENTER_LEFT_DUTY         32U
#define H_TRACK_LF04_CENTER_RIGHT_DUTY        34U
#define H_TRACK_LF04_SMALL_INNER_DUTY         24U
#define H_TRACK_LF04_SMALL_OUTER_DUTY         43U
#define H_TRACK_LF04_BIG_INNER_DUTY           15U
#define H_TRACK_LF04_BIG_OUTER_DUTY           53U
#define H_TRACK_LF04_SHARP_INNER_DUTY          0U
#define H_TRACK_LF04_SHARP_OUTER_DUTY         63U
#define H_TRACK_LF04_APPROACH_DUTY            17U
#define H_TRACK_LF04_FINISH_DUTY              10U

#define H_TRACK_LF04_MARKER_CONFIRM_MS        20U
#define H_TRACK_LF04_ENCODER_COUNTS_PER_CM    64U

/* Per-wheel denominators are counts per 100 cm. Keeping them separate avoids
 * turning the measured 2.8% encoder-scale difference into a lap-distance
 * error. The application converts each wheel to 0.01 cm, then averages. */
#define H_TRACK_LEFT_ENCODER_COUNTS_PER_100_CM   6445U
#define H_TRACK_RIGHT_ENCODER_COUNTS_PER_100_CM  6267U

/*
 * The diagrammed centre-line lap is:
 *   2 * 150 cm + 2 * pi * 50 cm = 614.159 cm.
 * The last 30 cm is followed at reduced duty, then a 100 ms TB6612 short
 * brake is applied once. Adjust only H_TRACK_ENCODER_LAP_STOP_CM_X100 after
 * measuring the signed A-point error on repeated full laps.
 */
#define H_TRACK_ENCODER_LAP_STOP_CM_X100     64446U
#define H_TRACK_ENCODER_SLOW_WINDOW_CM_X100   3000U
#define H_TRACK_ENCODER_SLOW_DUTY_PERCENT       55U
#define H_TRACK_ENCODER_STOP_BRAKE_MS           100U
#define H_TRACK_ENCODER_APPROX_LAP_COUNTS      39035U

/* Sensor-crossing to the desired car parking reference.  Leave at zero for
 * the first safe lap if A-marker parking is restored in a later revision. */
#define H_TRACK_LF04_STOP_OFFSET_CM            0U
#define H_TRACK_LF04_STOP_SLOW_WINDOW_CM      12U

/* Older eight-channel diagnostic configuration. */

#define H_TRACK_CENTER_LEFT_DUTY              17U
#define H_TRACK_CENTER_RIGHT_DUTY             19U

#define H_TRACK_SINGLE_INNER_DUTY             10U /* X4 or X5 */
#define H_TRACK_SINGLE_OUTER_DUTY             33U
#define H_TRACK_INNER_PAIR_INNER_DUTY         10U /* X3/X4 or X5/X6 */
#define H_TRACK_INNER_PAIR_OUTER_DUTY         43U
#define H_TRACK_MIDDLE_PAIR_INNER_DUTY         5U /* X2/X3 or X6/X7 */
#define H_TRACK_MIDDLE_PAIR_OUTER_DUTY        51U
#define H_TRACK_OUTER_PAIR_INNER_DUTY          7U /* X1/X2 or X7/X8 */
#define H_TRACK_OUTER_PAIR_OUTER_DUTY         61U
#define H_TRACK_EXTREME_INNER_DUTY             0U /* X1 or X8 */
#define H_TRACK_EXTREME_OUTER_DUTY            80U

#endif /* H_TRACK_PWM_CONFIG_H_ */
