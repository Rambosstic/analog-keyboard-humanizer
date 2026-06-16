#ifndef HUMANIZER_H
#define HUMANIZER_H

#include <stdint.h>

typedef struct {
    uint32_t rng_state;
    
    // Persistent angle offset memories to enable slow, smooth frame-to-frame drifting
    float wander_angle_l;
    float wander_angle_r;
    
    // Imperfect centering states
    int16_t drift_lx;
    int16_t drift_ly;
    int16_t drift_rx;
    int16_t drift_ry;
    uint32_t last_drift_time;
} Humanizer;

void humanizer_init(Humanizer* h);
void humanizer_process(Humanizer* h, int16_t* lx, int16_t* ly, int16_t* rx, int16_t* ry, uint16_t angle_spread, uint16_t deflection_scale, uint16_t deadzone);

#endif
