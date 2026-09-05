#ifndef COMPETITION_UI_H
#define COMPETITION_UI_H

#include <stdint.h>

enum {
    COMPETITION_KEY_START = 1U << 0,
    COMPETITION_KEY_FUNCTION2 = 1U << 1,
    COMPETITION_KEY_FUNCTION3 = 1U << 2,
    COMPETITION_KEY_FUNCTION456 = 1U << 3,
    COMPETITION_KEY_RESERVED = 1U << 4,
};

typedef enum {
    COMPETITION_FUNCTION_NONE = 0,
    COMPETITION_FUNCTION_TASK2,
    COMPETITION_FUNCTION_TASK3,
    COMPETITION_FUNCTION_TASK456,
} CompetitionFunction;

typedef enum {
    COMPETITION_UI_SELECT = 0,
    COMPETITION_UI_READY,
    COMPETITION_UI_RUNNING,
} CompetitionUiState;

typedef enum {
    COMPETITION_ACTION_NONE = 0,
    COMPETITION_ACTION_START_TASK,
    COMPETITION_ACTION_EMERGENCY_STOP,
} CompetitionUiAction;

typedef struct {
    CompetitionUiState state;
    CompetitionFunction selectedFunction;
} CompetitionUi;

/* Call once every 10 ms with debounced inputs. */
void CompetitionUi_init(CompetitionUi *ui);
CompetitionUiAction CompetitionUi_update(
    CompetitionUi *ui, uint32_t pressedEdges, uint32_t heldKeys);
void CompetitionUi_finishTask(CompetitionUi *ui);

#endif
