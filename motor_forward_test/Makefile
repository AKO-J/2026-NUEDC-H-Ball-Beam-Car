# Current H-task build entry points only.
# Historical one-off programs are preserved under archive/ and intentionally
# have no Make targets, so they cannot be selected by mistake during testing.

SDK ?= $(HOME)/ti/mspm0_sdk_2_11_00_07
COMPILER ?= $(HOME)/ti/ti_cgt_arm_llvm_4.0.2.LTS
CC := $(COMPILER)/bin/tiarmclang
HOST_CC ?= cc

DRIVERLIB := $(SDK)/source/ti/driverlib/lib/ticlang/m0p/mspm0g1x0x_g3x0x/driverlib.a
LINKER_CMD := $(SDK)/source/ti/devices/msp/m0p/linker_files/ticlang/mspm0g3507.cmd
STARTUP := $(SDK)/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c
BUILD_DIR := build

LINE_FOLLOW_TARGET := $(BUILD_DIR)/line_follow_test.out
LINE_FOLLOW_OBJECTS := $(BUILD_DIR)/h_track_task2.o \
	$(BUILD_DIR)/h_track_controller.o $(BUILD_DIR)/h_track_finish.o \
	$(BUILD_DIR)/speed_matcher.o \
	$(BUILD_DIR)/ir_line_sensor.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/icm42688.o $(BUILD_DIR)/imu_yaw.o $(BUILD_DIR)/motor_driver.o \
	$(BUILD_DIR)/encoder.o $(BUILD_DIR)/startup_mspm0g350x_ticlang.o

IR_READ_TARGET := $(BUILD_DIR)/ir_line_sensor_read_test.out
IR_READ_OBJECTS := $(BUILD_DIR)/ir_line_sensor_read_test.o \
	$(BUILD_DIR)/ir_line_sensor.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

ICM_TEST_TARGET := $(BUILD_DIR)/icm42688_test.out
ICM_TEST_OBJECTS := $(BUILD_DIR)/icm42688_test.o $(BUILD_DIR)/icm42688.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

ENCODER_DISTANCE_CAL_TARGET := $(BUILD_DIR)/encoder_distance_calibration_test.out
ENCODER_DISTANCE_CAL_OBJECTS := $(BUILD_DIR)/encoder_distance_calibration_test.o \
	$(BUILD_DIR)/motor_driver.o $(BUILD_DIR)/encoder.o \
	$(BUILD_DIR)/ssd1306_oled.o $(BUILD_DIR)/startup_mspm0g350x_ticlang.o

ICM_YAW_TEST_TARGET := $(BUILD_DIR)/icm_yaw_calibration_test.out
ICM_YAW_TEST_OBJECTS := $(BUILD_DIR)/icm_yaw_calibration_test.o \
	$(BUILD_DIR)/imu_yaw.o $(BUILD_DIR)/icm42688.o \
	$(BUILD_DIR)/ssd1306_oled.o $(BUILD_DIR)/startup_mspm0g350x_ticlang.o

MOTOR_IMU_TEST_TARGET := $(BUILD_DIR)/motor_imu_interference_test.out
MOTOR_IMU_TEST_OBJECTS := $(BUILD_DIR)/motor_imu_interference_test.o \
	$(BUILD_DIR)/imu_yaw.o $(BUILD_DIR)/icm42688.o \
	$(BUILD_DIR)/motor_driver.o $(BUILD_DIR)/encoder.o \
	$(BUILD_DIR)/ssd1306_oled.o $(BUILD_DIR)/startup_mspm0g350x_ticlang.o

STEPPER_JOG_TARGET := $(BUILD_DIR)/stepper_jog_test.out
STEPPER_JOG_OBJECTS := $(BUILD_DIR)/stepper_jog_test.o \
	$(BUILD_DIR)/stepper_beam.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

STEPPER_90_TARGET := $(BUILD_DIR)/stepper_fixed_90_test.out
STEPPER_90_OBJECTS := $(BUILD_DIR)/stepper_fixed_90_test.o \
	$(BUILD_DIR)/stepper_beam.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

STEPPER_FINE_JOG_TARGET := $(BUILD_DIR)/stepper_fine_jog_test.out
STEPPER_FINE_JOG_OBJECTS := $(BUILD_DIR)/stepper_fine_jog_test.o \
	$(BUILD_DIR)/stepper_beam.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

BALL_BEAM_VISION_TARGET := $(BUILD_DIR)/ball_beam_vision_control_test.out
BALL_BEAM_VISION_OBJECTS := $(BUILD_DIR)/ball_beam_vision_control_test.o \
	$(BUILD_DIR)/ball_beam_controller.o $(BUILD_DIR)/beam_calibration.o \
	$(BUILD_DIR)/beam_level_reference.o \
	$(BUILD_DIR)/vision_ball_protocol.o $(BUILD_DIR)/vision_uart.o \
	$(BUILD_DIR)/diagnostic_uart.o \
	$(BUILD_DIR)/stepper_beam.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

BALL_BEAM_TASK3_TARGET := $(BUILD_DIR)/ball_beam_task3_test.out
BALL_BEAM_TASK3_OBJECTS := $(BUILD_DIR)/ball_beam_task3_test.o \
	$(BUILD_DIR)/ball_beam_controller.o $(BUILD_DIR)/beam_calibration.o \
	$(BUILD_DIR)/beam_level_reference.o \
	$(BUILD_DIR)/vision_ball_protocol.o $(BUILD_DIR)/vision_uart.o \
	$(BUILD_DIR)/diagnostic_uart.o \
	$(BUILD_DIR)/stepper_beam.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

TASK4_BASELINE_TARGET := $(BUILD_DIR)/task4_baseline_test.out
TASK4_BASELINE_OBJECTS := $(BUILD_DIR)/task4_baseline_test.o \
	$(BUILD_DIR)/ball_beam_controller.o $(BUILD_DIR)/beam_calibration.o \
	$(BUILD_DIR)/beam_level_reference.o \
	$(BUILD_DIR)/vision_ball_protocol.o $(BUILD_DIR)/vision_uart.o \
	$(BUILD_DIR)/diagnostic_uart.o $(BUILD_DIR)/task4_baseline_profile.o \
	$(BUILD_DIR)/motor_driver.o $(BUILD_DIR)/encoder.o $(BUILD_DIR)/icm42688.o \
	$(BUILD_DIR)/stepper_beam.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

JOINT_LINE_BALANCE_TARGET := $(BUILD_DIR)/joint_line_balance_test.out
JOINT_LINE_BALANCE_OBJECTS := $(BUILD_DIR)/joint_line_balance_test.o \
	$(BUILD_DIR)/joint_line_balance_control.o $(BUILD_DIR)/ball_beam_controller.o \
	$(BUILD_DIR)/h_track_controller.o \
	$(BUILD_DIR)/beam_calibration.o \
	$(BUILD_DIR)/vision_ball_protocol.o $(BUILD_DIR)/vision_uart.o \
	$(BUILD_DIR)/diagnostic_uart.o $(BUILD_DIR)/motor_driver.o \
	$(BUILD_DIR)/encoder.o $(BUILD_DIR)/icm42688.o $(BUILD_DIR)/ir_line_sensor.o \
	$(BUILD_DIR)/stepper_beam.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

TIMED_LAP_TARGET := $(BUILD_DIR)/timed_lap_heading_capture_test.out
TIMED_LAP_OBJECTS := $(BUILD_DIR)/h_track_task2_timed_lap.o \
	$(BUILD_DIR)/h_track_controller.o $(BUILD_DIR)/h_track_finish.o \
	$(BUILD_DIR)/speed_matcher.o \
	$(BUILD_DIR)/ir_line_sensor.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/icm42688.o $(BUILD_DIR)/imu_yaw.o $(BUILD_DIR)/motor_driver.o \
	$(BUILD_DIR)/encoder.o $(BUILD_DIR)/startup_mspm0g350x_ticlang.o

STAGED_YAW_TARGET := $(BUILD_DIR)/staged_yaw_stop_test.out
STAGED_YAW_OBJECTS := $(BUILD_DIR)/h_track_task2_staged_yaw.o \
	$(BUILD_DIR)/h_track_controller.o $(BUILD_DIR)/h_track_finish.o \
	$(BUILD_DIR)/speed_matcher.o \
	$(BUILD_DIR)/ir_line_sensor.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/icm42688.o $(BUILD_DIR)/imu_yaw.o $(BUILD_DIR)/motor_driver.o \
	$(BUILD_DIR)/encoder.o $(BUILD_DIR)/startup_mspm0g350x_ticlang.o

H_TRACK_CONTROLLER_TEST := $(BUILD_DIR)/h_track_controller_test
H_TRACK_FINISH_TEST := $(BUILD_DIR)/h_track_finish_test
IR_LINE_SENSOR_MAPPING_TEST := $(BUILD_DIR)/ir_line_sensor_mapping_test
SPEED_MATCHER_TEST := $(BUILD_DIR)/speed_matcher_test
BALL_BEAM_VISION_HOST_TEST := $(BUILD_DIR)/ball_beam_vision_test
COMPETITION_UI_HOST_TEST := $(BUILD_DIR)/competition_ui_test
TASK4_BASELINE_PROFILE_TEST := $(BUILD_DIR)/task4_baseline_profile_test
JOINT_LINE_BALANCE_HOST_TEST := $(BUILD_DIR)/joint_line_balance_control_test
COMPETITION_KEYS_OLED_TARGET := $(BUILD_DIR)/competition_keys_oled_test.out
COMPETITION_KEYS_OLED_OBJECTS := $(BUILD_DIR)/competition_keys_oled_test.o \
	$(BUILD_DIR)/competition_ui.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o
COMPETITION_UNIFIED_TARGET := $(BUILD_DIR)/competition_unified.out
COMPETITION_UNIFIED_OBJECTS := $(BUILD_DIR)/competition_selector_main.o \
	$(BUILD_DIR)/competition_ui.o $(BUILD_DIR)/h_track_task2_unified.o \
	$(BUILD_DIR)/ball_beam_task3_unified.o $(BUILD_DIR)/joint456_unified.o \
	$(BUILD_DIR)/joint_line_balance_control.o \
	$(BUILD_DIR)/h_track_controller.o $(BUILD_DIR)/h_track_finish.o \
	$(BUILD_DIR)/speed_matcher.o $(BUILD_DIR)/ball_beam_controller.o \
	$(BUILD_DIR)/beam_calibration.o $(BUILD_DIR)/beam_level_reference.o \
	$(BUILD_DIR)/vision_ball_protocol.o $(BUILD_DIR)/vision_uart.o \
	$(BUILD_DIR)/diagnostic_uart.o $(BUILD_DIR)/ir_line_sensor.o \
	$(BUILD_DIR)/icm42688.o $(BUILD_DIR)/imu_yaw.o \
	$(BUILD_DIR)/motor_driver.o $(BUILD_DIR)/encoder.o \
	$(BUILD_DIR)/stepper_beam.o $(BUILD_DIR)/ssd1306_oled.o \
	$(BUILD_DIR)/startup_mspm0g350x_ticlang.o

CFLAGS := -I. -D__MSPM0G3507__ -O2 -gdwarf-3 \
	-I$(SDK)/source/third_party/CMSIS/Core/Include -I$(SDK)/source \
	-mcpu=cortex-m0plus -march=thumbv6m -mfloat-abi=soft -mthumb -Wall

.PHONY: all help line-follow ir-line-sensor-read-test icm42688-test \
	encoder-distance-calibration-test \
	icm-yaw-calibration-test motor-imu-interference-test \
	stepper-jog-test stepper-fixed-90-test stepper-fine-jog-test \
	ball-beam-vision-control-test ball-beam-task3-test \
	task4-baseline-test joint-line-balance-test \
	competition-keys-oled-test competition-unified \
	timed-lap-heading-capture-test \
	staged-yaw-stop-test \
	h-track-controller-test h-track-finish-test \
	ir-line-sensor-mapping-test speed-matcher-test \
	task4-baseline-profile-test joint-line-balance-control-test \
	host-tests check-host flash-ir-line-sensor-read-test flash-line-follow \
	flash-ball-beam-vision-control-test flash-ball-beam-task3-test \
	flash-task4-baseline-test flash-joint-line-balance-test \
	flash-competition-unified

all: line-follow

help:
	@echo "Current H-task targets:"
	@echo "  make line-follow                 formal LF04 task-2 firmware"
	@echo "  make ir-line-sensor-read-test    safe LF04 + OLED readout"
	@echo "  make icm42688-test               safe ICM42688 I2C test"
	@echo "  make encoder-distance-calibration-test PB7-start 6000-count straight test"
	@echo "  make icm-yaw-calibration-test    safe ICM + OLED 360-degree calibration"
	@echo "  make motor-imu-interference-test motor vibration / yaw drift calibration"
	@echo "  make stepper-jog-test            X42S + OLED manual-zero/jog test"
	@echo "  make stepper-fixed-90-test       X42S START-triggered +/-90-degree test"
	@echo "  make stepper-fine-jog-test       X42S START-arm, UP/DOWN 100-pulse test"
	@echo "  make ball-beam-vision-control-test K230 UART + calibrated X42S control"
	@echo "  make ball-beam-task3-test       task 3: 0 -> +5 cm -> -5 cm sequence"
	@echo "  make task4-baseline-test        task 4: centred beam + low-speed AB baseline"
	@echo "  make competition-keys-oled-test safe five-key selector + OLED test"
	@echo "  make competition-unified        S2/S3/S4 unified competition firmware"
	@echo "  make flash-competition-unified  flash unified firmware, do not run it"
	@echo "  make timed-lap-heading-capture-test stop at 12.000 s and latch HDG"
	@echo "  make staged-yaw-stop-test          145/160-degree staged slow, 190-degree stop"
	@echo "  make check-host                  run controller/mapping unit tests"
	@echo "  make task4-baseline-profile-test verify low-speed AB motion profile"
	@echo "  make flash-line-follow           flash formal firmware, do not run it"

line-follow: $(LINE_FOLLOW_TARGET)

ir-line-sensor-read-test: $(IR_READ_TARGET)

icm42688-test: $(ICM_TEST_TARGET)

encoder-distance-calibration-test: $(ENCODER_DISTANCE_CAL_TARGET)

icm-yaw-calibration-test: $(ICM_YAW_TEST_TARGET)

motor-imu-interference-test: $(MOTOR_IMU_TEST_TARGET)

stepper-jog-test: $(STEPPER_JOG_TARGET)

stepper-fixed-90-test: $(STEPPER_90_TARGET)

stepper-fine-jog-test: $(STEPPER_FINE_JOG_TARGET)

ball-beam-vision-control-test: $(BALL_BEAM_VISION_TARGET)

ball-beam-task3-test: $(BALL_BEAM_TASK3_TARGET)

task4-baseline-test: $(TASK4_BASELINE_TARGET)

joint-line-balance-test: $(JOINT_LINE_BALANCE_TARGET)

competition-keys-oled-test: $(COMPETITION_KEYS_OLED_TARGET)

competition-unified: $(COMPETITION_UNIFIED_TARGET)

timed-lap-heading-capture-test: $(TIMED_LAP_TARGET)

staged-yaw-stop-test: $(STAGED_YAW_TARGET)

h-track-controller-test: $(H_TRACK_CONTROLLER_TEST)

h-track-finish-test: $(H_TRACK_FINISH_TEST)

ir-line-sensor-mapping-test: $(IR_LINE_SENSOR_MAPPING_TEST)

speed-matcher-test: $(SPEED_MATCHER_TEST)

task4-baseline-profile-test: $(TASK4_BASELINE_PROFILE_TEST)

joint-line-balance-control-test: $(JOINT_LINE_BALANCE_HOST_TEST)

host-tests: h-track-controller-test h-track-finish-test \
	ir-line-sensor-mapping-test speed-matcher-test \
	$(BALL_BEAM_VISION_HOST_TEST) $(COMPETITION_UI_HOST_TEST) \
	$(TASK4_BASELINE_PROFILE_TEST) $(JOINT_LINE_BALANCE_HOST_TEST)

check-host: host-tests
	$(H_TRACK_CONTROLLER_TEST)
	$(H_TRACK_FINISH_TEST)
	$(IR_LINE_SENSOR_MAPPING_TEST)
	$(SPEED_MATCHER_TEST)
	$(BALL_BEAM_VISION_HOST_TEST)
	$(COMPETITION_UI_HOST_TEST)
	$(TASK4_BASELINE_PROFILE_TEST)
	$(JOINT_LINE_BALANCE_HOST_TEST)

flash-ir-line-sensor-read-test: $(IR_READ_TARGET)
	./flash_xds110.sh --yes $(IR_READ_TARGET)

flash-line-follow: $(LINE_FOLLOW_TARGET)
	./flash_xds110.sh --yes $(LINE_FOLLOW_TARGET)

flash-ball-beam-vision-control-test: $(BALL_BEAM_VISION_TARGET)
	./flash_xds110.sh --yes $(BALL_BEAM_VISION_TARGET)

flash-ball-beam-task3-test: $(BALL_BEAM_TASK3_TARGET)
	./flash_xds110.sh --yes $(BALL_BEAM_TASK3_TARGET)

flash-task4-baseline-test: $(TASK4_BASELINE_TARGET)
	./flash_xds110.sh --yes $(TASK4_BASELINE_TARGET)

flash-joint-line-balance-test: $(JOINT_LINE_BALANCE_TARGET)
	./flash_xds110.sh --yes $(JOINT_LINE_BALANCE_TARGET)

flash-competition-unified: $(COMPETITION_UNIFIED_TARGET)
	./flash_xds110.sh --yes $(COMPETITION_UNIFIED_TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/h_track_task2.o: h_track_task2.c ir_line_sensor.h h_track_controller.h h_track_finish.h h_track_pwm_config.h speed_matcher.h icm42688.h imu_yaw.h ssd1306_oled.h motor_driver.h encoder.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/competition_ui.o: competition_ui.c competition_ui.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/competition_keys_oled_test.o: tests/bringup/competition_keys_oled_test.c competition_ui.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/competition_selector_main.o: competition_selector_main.c competition_ui.h competition_entries.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/h_track_task2_unified.o: h_track_task2.c competition_entries.h ir_line_sensor.h h_track_controller.h h_track_finish.h h_track_pwm_config.h speed_matcher.h icm42688.h imu_yaw.h ssd1306_oled.h motor_driver.h encoder.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DCOMPETITION_UNIFIED_ENTRY=1 -c $< -o $@

$(BUILD_DIR)/ball_beam_task3_unified.o: tests/bringup/ball_beam_vision_control_test.c competition_entries.h ball_beam_controller.h beam_calibration.h beam_level_reference.h diagnostic_uart.h vision_uart.h stepper_beam.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DCOMPETITION_UNIFIED_ENTRY=1 -DBALL_BEAM_TASK3_SEQUENCE=1 -c $< -o $@

$(BUILD_DIR)/joint456_unified.o: tests/bringup/joint_line_balance_test.c competition_entries.h joint_line_balance_control.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DCOMPETITION_UNIFIED_ENTRY=1 -c $< -o $@

$(BUILD_DIR)/h_track_task2_timed_lap.o: h_track_task2.c ir_line_sensor.h h_track_controller.h h_track_finish.h h_track_pwm_config.h speed_matcher.h icm42688.h imu_yaw.h ssd1306_oled.h motor_driver.h encoder.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DH_TRACK_TEST_TIMED_LAP=1 -c $< -o $@

$(BUILD_DIR)/h_track_task2_staged_yaw.o: h_track_task2.c ir_line_sensor.h h_track_controller.h h_track_finish.h h_track_pwm_config.h speed_matcher.h icm42688.h imu_yaw.h ssd1306_oled.h motor_driver.h encoder.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DH_TRACK_TEST_STAGED_YAW_STOP=1 -c $< -o $@

$(BUILD_DIR)/h_track_controller.o: h_track_controller.c h_track_controller.h h_track_pwm_config.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/h_track_finish.o: h_track_finish.c h_track_finish.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/speed_matcher.o: speed_matcher.c speed_matcher.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/ir_line_sensor.o: ir_line_sensor.c ir_line_sensor.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/ssd1306_oled.o: ssd1306_oled.c ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/icm42688.o: icm42688.c icm42688.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/motor_driver.o: motor_driver.c motor_driver.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/encoder.o: encoder.c encoder.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/startup_mspm0g350x_ticlang.o: $(STARTUP) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/ir_line_sensor_read_test.o: tests/bringup/ir_line_sensor_read_test.c ir_line_sensor.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/icm42688_test.o: tests/bringup/icm42688_test.c icm42688.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/encoder_distance_calibration_test.o: tests/bringup/encoder_distance_calibration_test.c motor_driver.h encoder.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/icm_yaw_calibration_test.o: tests/bringup/icm_yaw_calibration_test.c imu_yaw.h icm42688.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/motor_imu_interference_test.o: tests/bringup/motor_imu_interference_test.c imu_yaw.h icm42688.h motor_driver.h encoder.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stepper_jog_test.o: archive/legacy_bringup/stepper_jog_test.c stepper_beam.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stepper_fixed_90_test.o: archive/legacy_bringup/stepper_fixed_90_test.c stepper_beam.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stepper_fine_jog_test.o: archive/legacy_bringup/stepper_fine_jog_test.c stepper_beam.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/ball_beam_vision_control_test.o: tests/bringup/ball_beam_vision_control_test.c ball_beam_controller.h beam_calibration.h beam_level_reference.h diagnostic_uart.h vision_uart.h stepper_beam.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/ball_beam_task3_test.o: tests/bringup/ball_beam_vision_control_test.c ball_beam_controller.h beam_calibration.h beam_level_reference.h diagnostic_uart.h vision_uart.h stepper_beam.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DBALL_BEAM_TASK3_SEQUENCE=1 -c $< -o $@

$(BUILD_DIR)/task4_baseline_test.o: tests/bringup/task4_baseline_test.c tests/bringup/ball_beam_vision_control_test.c ball_beam_controller.h beam_calibration.h beam_level_reference.h diagnostic_uart.h vision_uart.h stepper_beam.h ssd1306_oled.h task4_baseline_profile.h motor_driver.h encoder.h icm42688.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stepper_beam.o: stepper_beam.c stepper_beam.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/ball_beam_controller.o: ball_beam_controller.c ball_beam_controller.h beam_calibration.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/beam_calibration.o: beam_calibration.c beam_calibration.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/beam_level_reference.o: beam_level_reference.c beam_level_reference.h beam_calibration.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/vision_ball_protocol.o: vision_ball_protocol.c vision_ball_protocol.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/vision_uart.o: vision_uart.c vision_uart.h vision_ball_protocol.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/diagnostic_uart.o: diagnostic_uart.c diagnostic_uart.h stepper_beam.h vision_ball_protocol.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/task4_baseline_profile.o: task4_baseline_profile.c task4_baseline_profile.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/joint_line_balance_control.o: joint_line_balance_control.c joint_line_balance_control.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/joint_line_balance_test.o: tests/bringup/joint_line_balance_test.c joint_line_balance_control.h ball_beam_controller.h ssd1306_oled.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/imu_yaw.o: imu_yaw.c imu_yaw.h icm42688.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(LINE_FOLLOW_TARGET): $(LINE_FOLLOW_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(LINE_FOLLOW_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/line_follow_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(IR_READ_TARGET): $(IR_READ_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(IR_READ_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/ir_line_sensor_read_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(ICM_TEST_TARGET): $(ICM_TEST_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(ICM_TEST_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/icm42688_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(ENCODER_DISTANCE_CAL_TARGET): $(ENCODER_DISTANCE_CAL_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(ENCODER_DISTANCE_CAL_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/encoder_distance_calibration_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(ICM_YAW_TEST_TARGET): $(ICM_YAW_TEST_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(ICM_YAW_TEST_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/icm_yaw_calibration_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(MOTOR_IMU_TEST_TARGET): $(MOTOR_IMU_TEST_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(MOTOR_IMU_TEST_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/motor_imu_interference_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(STEPPER_JOG_TARGET): $(STEPPER_JOG_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(STEPPER_JOG_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/stepper_jog_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(STEPPER_90_TARGET): $(STEPPER_90_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(STEPPER_90_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/stepper_fixed_90_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(STEPPER_FINE_JOG_TARGET): $(STEPPER_FINE_JOG_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(STEPPER_FINE_JOG_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/stepper_fine_jog_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(TIMED_LAP_TARGET): $(TIMED_LAP_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(TIMED_LAP_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/timed_lap_heading_capture_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(STAGED_YAW_TARGET): $(STAGED_YAW_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(STAGED_YAW_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/staged_yaw_stop_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(H_TRACK_CONTROLLER_TEST): tests/host/h_track_controller_test.c h_track_controller.c h_track_controller.h | $(BUILD_DIR)
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -I. tests/host/h_track_controller_test.c h_track_controller.c -o $@

$(H_TRACK_FINISH_TEST): tests/host/h_track_finish_test.c h_track_finish.c h_track_finish.h | $(BUILD_DIR)
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -I. tests/host/h_track_finish_test.c h_track_finish.c -o $@

$(IR_LINE_SENSOR_MAPPING_TEST): tests/host/ir_line_sensor_mapping_test.c ir_line_sensor.c ir_line_sensor.h | $(BUILD_DIR)
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -DIR_LINE_SENSOR_HOST_ONLY -I. tests/host/ir_line_sensor_mapping_test.c ir_line_sensor.c -o $@

$(SPEED_MATCHER_TEST): tests/host/speed_matcher_test.c speed_matcher.c speed_matcher.h | $(BUILD_DIR)
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -I. tests/host/speed_matcher_test.c speed_matcher.c -o $@

$(BALL_BEAM_VISION_HOST_TEST): tests/host/ball_beam_vision_test.c ball_beam_controller.c ball_beam_controller.h beam_calibration.c beam_calibration.h beam_level_reference.c beam_level_reference.h vision_ball_protocol.c vision_ball_protocol.h | $(BUILD_DIR)
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -I. \
		tests/host/ball_beam_vision_test.c ball_beam_controller.c \
		beam_calibration.c beam_level_reference.c vision_ball_protocol.c -o $@

$(COMPETITION_UI_HOST_TEST): tests/host/competition_ui_test.c competition_ui.c competition_ui.h | $(BUILD_DIR)
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -I. \
		tests/host/competition_ui_test.c competition_ui.c -o $@

$(TASK4_BASELINE_PROFILE_TEST): tests/host/task4_baseline_profile_test.c task4_baseline_profile.c task4_baseline_profile.h | $(BUILD_DIR)
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -I. \
		tests/host/task4_baseline_profile_test.c task4_baseline_profile.c -o $@

$(JOINT_LINE_BALANCE_HOST_TEST): tests/host/joint_line_balance_control_test.c joint_line_balance_control.c joint_line_balance_control.h | $(BUILD_DIR)
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -I. \
		tests/host/joint_line_balance_control_test.c joint_line_balance_control.c -o $@

$(BALL_BEAM_VISION_TARGET): $(BALL_BEAM_VISION_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(BALL_BEAM_VISION_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/ball_beam_vision_control_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(BALL_BEAM_TASK3_TARGET): $(BALL_BEAM_TASK3_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(BALL_BEAM_TASK3_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/ball_beam_task3_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(TASK4_BASELINE_TARGET): $(TASK4_BASELINE_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(TASK4_BASELINE_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/task4_baseline_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(JOINT_LINE_BALANCE_TARGET): $(JOINT_LINE_BALANCE_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(JOINT_LINE_BALANCE_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/joint_line_balance_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(COMPETITION_KEYS_OLED_TARGET): $(COMPETITION_KEYS_OLED_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(COMPETITION_KEYS_OLED_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/competition_keys_oled_test.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@

$(COMPETITION_UNIFIED_TARGET): $(COMPETITION_UNIFIED_OBJECTS)
	$(CC) -Wl,-u,_c_int00 $(COMPETITION_UNIFIED_OBJECTS) -l$(DRIVERLIB) -L$(SDK)/source \
		$(LINKER_CMD) -Wl,-m,$(BUILD_DIR)/competition_unified.map \
		-Wl,--rom_model -Wl,--warn_sections -L$(COMPILER)/lib -llibc.a -o $@
