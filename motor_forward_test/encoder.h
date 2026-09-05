#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

/*
 * Debug-visible signed quadrature counts.
 * A positive/negative sign only reflects the electrical phase order; the
 * direction convention is calibrated after the first readout.
 */
extern volatile int32_t g_motorA_encoder_count;
extern volatile int32_t g_motorB_encoder_count;
extern volatile uint32_t g_motorA_encoder_edges;
extern volatile uint32_t g_motorB_encoder_edges;
extern volatile uint32_t g_motorA_encA_edges;
extern volatile uint32_t g_motorA_encB_edges;
extern volatile uint32_t g_motorB_encA_edges;
extern volatile uint32_t g_motorB_encB_edges;
extern volatile uint32_t g_motorA_encoder_invalid_transitions;
extern volatile uint32_t g_motorB_encoder_invalid_transitions;

void Encoder_init(void);
void Encoder_resetCounts(void);
/*
 * Encoder_init enables double-edge GPIO interrupts for all four A/B inputs.
 * Encoder_sample remains available for diagnostics and takes an atomic
 * snapshot; normal driving no longer depends on how often it is called.
 */
void Encoder_sample(void);

#endif /* ENCODER_H_ */
