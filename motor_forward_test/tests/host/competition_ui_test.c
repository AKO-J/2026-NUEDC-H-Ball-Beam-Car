#include <assert.h>
#include <stdio.h>

#include "competition_ui.h"

int main(void)
{
    CompetitionUi ui;
    CompetitionUi_init(&ui);
    assert(ui.state == COMPETITION_UI_SELECT);

    (void) CompetitionUi_update(&ui, COMPETITION_KEY_FUNCTION2,
                                COMPETITION_KEY_FUNCTION2);
    assert(ui.state == COMPETITION_UI_READY &&
           ui.selectedFunction == COMPETITION_FUNCTION_TASK2);

    (void) CompetitionUi_update(&ui, COMPETITION_KEY_FUNCTION3,
                                COMPETITION_KEY_FUNCTION3);
    assert(ui.state == COMPETITION_UI_READY &&
           ui.selectedFunction == COMPETITION_FUNCTION_TASK3);

    assert(CompetitionUi_update(&ui, COMPETITION_KEY_START,
                                COMPETITION_KEY_START) ==
           COMPETITION_ACTION_START_TASK);
    assert(ui.state == COMPETITION_UI_RUNNING);
    (void) CompetitionUi_update(&ui, COMPETITION_KEY_FUNCTION456,
                                COMPETITION_KEY_FUNCTION456);
    assert(ui.selectedFunction == COMPETITION_FUNCTION_TASK3);

    assert(CompetitionUi_update(&ui, COMPETITION_KEY_START,
                                COMPETITION_KEY_START) ==
           COMPETITION_ACTION_EMERGENCY_STOP);
    assert(ui.state == COMPETITION_UI_READY);
    (void) CompetitionUi_update(&ui, COMPETITION_KEY_RESERVED,
                                COMPETITION_KEY_RESERVED);
    assert(ui.state == COMPETITION_UI_READY);

    (void) CompetitionUi_update(&ui, COMPETITION_KEY_FUNCTION456,
                                COMPETITION_KEY_FUNCTION456);
    assert(ui.selectedFunction == COMPETITION_FUNCTION_TASK456);

    CompetitionUi_finishTask(&ui);
    puts("competition_ui_test: PASS");
    return 0;
}
