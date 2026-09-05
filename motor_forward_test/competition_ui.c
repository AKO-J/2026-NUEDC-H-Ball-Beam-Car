#include "competition_ui.h"

void CompetitionUi_init(CompetitionUi *ui)
{
    ui->state = COMPETITION_UI_SELECT;
    ui->selectedFunction = COMPETITION_FUNCTION_NONE;
}

static CompetitionFunction selected_function(uint32_t pressedEdges)
{
    if ((pressedEdges & COMPETITION_KEY_FUNCTION2) != 0U)
        return COMPETITION_FUNCTION_TASK2;
    if ((pressedEdges & COMPETITION_KEY_FUNCTION3) != 0U)
        return COMPETITION_FUNCTION_TASK3;
    if ((pressedEdges & COMPETITION_KEY_FUNCTION456) != 0U)
        return COMPETITION_FUNCTION_TASK456;
    return COMPETITION_FUNCTION_NONE;
}

CompetitionUiAction CompetitionUi_update(
    CompetitionUi *ui, uint32_t pressedEdges, uint32_t heldKeys)
{
    const CompetitionFunction function = selected_function(pressedEdges);
    (void) heldKeys;

    /* START is always an immediate stop while a task is active. */
    if (ui->state == COMPETITION_UI_RUNNING) {
        if ((pressedEdges & COMPETITION_KEY_START) != 0U) {
            ui->state = COMPETITION_UI_READY;
            return COMPETITION_ACTION_EMERGENCY_STOP;
        }
        return COMPETITION_ACTION_NONE;
    }

    /* S5 is intentionally reserved. Every selected task retains its own
     * validated calibration sequence after the selector dispatches it. */
    if (function != COMPETITION_FUNCTION_NONE) {
        ui->selectedFunction = function;
        ui->state = COMPETITION_UI_READY;
        return COMPETITION_ACTION_NONE;
    }
    if (((pressedEdges & COMPETITION_KEY_START) != 0U) &&
        (ui->state == COMPETITION_UI_READY)) {
        ui->state = COMPETITION_UI_RUNNING;
        return COMPETITION_ACTION_START_TASK;
    }
    return COMPETITION_ACTION_NONE;
}

void CompetitionUi_finishTask(CompetitionUi *ui)
{
    if (ui->state == COMPETITION_UI_RUNNING)
        ui->state = COMPETITION_UI_READY;
}
