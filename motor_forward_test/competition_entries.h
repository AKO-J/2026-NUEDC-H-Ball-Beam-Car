#ifndef COMPETITION_ENTRIES_H
#define COMPETITION_ENTRIES_H

/* Each entry owns the MCU after selection and preserves its original
 * internal button/calibration state machine. These functions do not return. */
void Task2_run(void);
void Task3_run(void);
void Joint456_run(void);

#endif
