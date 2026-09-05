#include "encoder.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/*
 * Encoder wiring, generated and verified with encoder_pins.syscfg:
 *   E1A -> PB2  (PinCM15), E1B -> PB24 (PinCM52)
 *   E2A -> PB9  (PinCM26), E2B -> PA27 (PinCM60)
 */
#define MOTOR_A_ENC_A_PORT        GPIOB
#define MOTOR_A_ENC_A_PIN         DL_GPIO_PIN_2
#define MOTOR_A_ENC_A_IOMUX       IOMUX_PINCM15

#define MOTOR_A_ENC_B_PORT        GPIOB
#define MOTOR_A_ENC_B_PIN         DL_GPIO_PIN_24
#define MOTOR_A_ENC_B_IOMUX       IOMUX_PINCM52

#define MOTOR_B_ENC_A_PORT        GPIOB
#define MOTOR_B_ENC_A_PIN         DL_GPIO_PIN_9
#define MOTOR_B_ENC_A_IOMUX       IOMUX_PINCM26

#define MOTOR_B_ENC_B_PORT        GPIOA
#define MOTOR_B_ENC_B_PIN         DL_GPIO_PIN_27
#define MOTOR_B_ENC_B_IOMUX       IOMUX_PINCM60

#define MOTOR_A_GPIOB_ENCODER_PINS \
    (MOTOR_A_ENC_A_PIN | MOTOR_A_ENC_B_PIN)
#define MOTOR_B_GPIOB_ENCODER_PINS MOTOR_B_ENC_A_PIN
#define MOTOR_B_GPIOA_ENCODER_PINS MOTOR_B_ENC_B_PIN
#define ALL_GPIOB_ENCODER_PINS \
    (MOTOR_A_GPIOB_ENCODER_PINS | MOTOR_B_GPIOB_ENCODER_PINS)

/* Calibrated from the one-second dual-wheel forward test. */
#define MOTOR_A_FORWARD_COUNT_SIGN   1
#define MOTOR_B_FORWARD_COUNT_SIGN  -1

volatile int32_t g_motorA_encoder_count = 0;
volatile int32_t g_motorB_encoder_count = 0;
volatile uint32_t g_motorA_encoder_edges = 0;
volatile uint32_t g_motorB_encoder_edges = 0;
volatile uint32_t g_motorA_encA_edges = 0;
volatile uint32_t g_motorA_encB_edges = 0;
volatile uint32_t g_motorB_encA_edges = 0;
volatile uint32_t g_motorB_encB_edges = 0;
volatile uint32_t g_motorA_encoder_invalid_transitions = 0;
volatile uint32_t g_motorB_encoder_invalid_transitions = 0;

static uint8_t g_motorA_last_state;
static uint8_t g_motorB_last_state;
static uint8_t g_motorA_encA_last;
static uint8_t g_motorA_encB_last;
static uint8_t g_motorB_encA_last;
static uint8_t g_motorB_encB_last;

/* Index: (previous AB state << 2) | current AB state. */
static const int8_t k_quadrature_delta[16] = {
    0, 1, -1, 0,
   -1, 0,  0, 1,
    1, 0,  0,-1,
    0,-1,  1, 0
};

static uint8_t read_encoder_state(GPIO_Regs *portA, uint32_t pinA,
                                  GPIO_Regs *portB, uint32_t pinB)
{
    uint8_t state = 0U;

    if (DL_GPIO_readPins(portA, pinA) != 0U) {
        state |= 0x2U;
    }
    if (DL_GPIO_readPins(portB, pinB) != 0U) {
        state |= 0x1U;
    }
    return state;
}

static void update_encoder(uint8_t *lastState, uint8_t currentState,
                           volatile int32_t *count,
                           volatile uint32_t *edges,
                           volatile uint32_t *invalidTransitions,
                           int8_t forwardCountSign)
{
    const uint8_t transition = (uint8_t) ((*lastState << 2U) | currentState);
    const int8_t delta = k_quadrature_delta[transition];

    if ((delta == 0) && (currentState != *lastState)) {
        /* Both phases changed between samples: record but do not invent a step. */
        (*invalidTransitions)++;
    } else {
        *count += ((int32_t) delta * (int32_t) forwardCountSign);
        if (delta != 0) {
            (*edges)++;
        }
    }
    *lastState = currentState;
}

static void update_signal_edge(uint8_t *lastLevel, uint8_t currentLevel,
                               volatile uint32_t *edges)
{
    if (currentLevel != *lastLevel) {
        (*edges)++;
    }
    *lastLevel = currentLevel;
}

static void sample_encoders_unlocked(void)
{
    const uint8_t motorAState = read_encoder_state(MOTOR_A_ENC_A_PORT,
        MOTOR_A_ENC_A_PIN, MOTOR_A_ENC_B_PORT, MOTOR_A_ENC_B_PIN);
    const uint8_t motorBState = read_encoder_state(MOTOR_B_ENC_A_PORT,
        MOTOR_B_ENC_A_PIN, MOTOR_B_ENC_B_PORT, MOTOR_B_ENC_B_PIN);

    update_encoder(&g_motorA_last_state, motorAState, &g_motorA_encoder_count,
                   &g_motorA_encoder_edges,
                   &g_motorA_encoder_invalid_transitions,
                   MOTOR_A_FORWARD_COUNT_SIGN);
    update_encoder(&g_motorB_last_state, motorBState, &g_motorB_encoder_count,
                   &g_motorB_encoder_edges,
                   &g_motorB_encoder_invalid_transitions,
                   MOTOR_B_FORWARD_COUNT_SIGN);

    update_signal_edge(&g_motorA_encA_last, (motorAState >> 1U) & 0x1U,
                       &g_motorA_encA_edges);
    update_signal_edge(&g_motorA_encB_last, motorAState & 0x1U,
                       &g_motorA_encB_edges);
    update_signal_edge(&g_motorB_encA_last, (motorBState >> 1U) & 0x1U,
                       &g_motorB_encA_edges);
    update_signal_edge(&g_motorB_encB_last, motorBState & 0x1U,
                       &g_motorB_encB_edges);
}

static void reset_counts_unlocked(void)
{
    g_motorA_encoder_count = 0;
    g_motorB_encoder_count = 0;
    g_motorA_encoder_edges = 0;
    g_motorB_encoder_edges = 0;
    g_motorA_encA_edges = 0;
    g_motorA_encB_edges = 0;
    g_motorB_encA_edges = 0;
    g_motorB_encB_edges = 0;
    g_motorA_encoder_invalid_transitions = 0;
    g_motorB_encoder_invalid_transitions = 0;

    g_motorA_last_state = read_encoder_state(MOTOR_A_ENC_A_PORT,
        MOTOR_A_ENC_A_PIN, MOTOR_A_ENC_B_PORT, MOTOR_A_ENC_B_PIN);
    g_motorB_last_state = read_encoder_state(MOTOR_B_ENC_A_PORT,
        MOTOR_B_ENC_A_PIN, MOTOR_B_ENC_B_PORT, MOTOR_B_ENC_B_PIN);
    g_motorA_encA_last = (g_motorA_last_state >> 1U) & 0x1U;
    g_motorA_encB_last = g_motorA_last_state & 0x1U;
    g_motorB_encA_last = (g_motorB_last_state >> 1U) & 0x1U;
    g_motorB_encB_last = g_motorB_last_state & 0x1U;
}

void Encoder_init(void)
{
    DL_GPIO_initDigitalInputFeatures(MOTOR_A_ENC_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MOTOR_A_ENC_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MOTOR_B_ENC_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MOTOR_B_ENC_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /*
     * Count every rising and falling A/B transition in the GPIO ISR. The old
     * main-loop polling path missed transitions while IMU/OLED I/O was busy,
     * so the error grew with vehicle speed.
     */
    DL_GPIO_setLowerPinsPolarity(GPIOB,
        DL_GPIO_PIN_2_EDGE_RISE_FALL | DL_GPIO_PIN_9_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_24_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_27_EDGE_RISE_FALL);

    reset_counts_unlocked();
    DL_GPIO_clearInterruptStatus(GPIOB, ALL_GPIOB_ENCODER_PINS);
    DL_GPIO_clearInterruptStatus(GPIOA, MOTOR_B_GPIOA_ENCODER_PINS);
    DL_GPIO_enableInterrupt(GPIOB, ALL_GPIOB_ENCODER_PINS);
    DL_GPIO_enableInterrupt(GPIOA, MOTOR_B_GPIOA_ENCODER_PINS);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void Encoder_resetCounts(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    reset_counts_unlocked();
    DL_GPIO_clearInterruptStatus(GPIOB, ALL_GPIOB_ENCODER_PINS);
    DL_GPIO_clearInterruptStatus(GPIOA, MOTOR_B_GPIOA_ENCODER_PINS);
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

void Encoder_sample(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    sample_encoders_unlocked();
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

void GROUP1_IRQHandler(void)
{
    const uint32_t pendingB = DL_GPIO_getEnabledInterruptStatus(
        GPIOB, ALL_GPIOB_ENCODER_PINS);
    const uint32_t pendingA = DL_GPIO_getEnabledInterruptStatus(
        GPIOA, MOTOR_B_GPIOA_ENCODER_PINS);

    /*
     * Clear the latched edge first. If another edge arrives while sampling,
     * it remains pending and causes another ISR instead of being erased.
     */
    if (pendingB != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOB, pendingB);
    }
    if (pendingA != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOA, pendingA);
    }
    if ((pendingA | pendingB) != 0U) {
        sample_encoders_unlocked();
    }
}
