#ifndef HUMANIZER_H
#define HUMANIZER_H

#include <stdint.h>

typedef struct {
    // smoothing state
    float prev_x_l, prev_y_l;
    float prev_x_r, prev_y_r;

    // Physics Oscillator State (Position AND Velocity) for the "Organic Wave"
    float wob_p_l, wob_v_l;
    float wob_p_r, wob_v_r;
    
    // OU bias state — the slowly wandering "hand bias"
    float bias_l, bias_r;
    
    // gate random-walk state, per stick
    float gate_l, gate_r;
    
    // non-stationary modulation: slowly drifting sigma scaler per stick
    float sig_l, sig_r;

    // timing
    uint64_t last_us;
    int      have_last;

    // rng
    uint32_t rng;
} Humanizer;

void humanizer_init(Humanizer* h);

// passthrough != 0 bypasses ALL processing (raw passthrough for A/B testing)
void humanizer_process(Humanizer* h, int16_t* lx, int16_t* ly, int16_t* rx, int16_t* ry,
                       uint16_t circ_error, uint16_t axis_deviation,
                       uint16_t smoothing_rate, uint16_t gate_level,
                       uint16_t tilt_deg, uint16_t passthrough);

#endif
