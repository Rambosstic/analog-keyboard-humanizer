#ifndef HUMANIZER_H
#define HUMANIZER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // Continuous 2D Target-Seeking State
    float drift_x;
    float drift_y;
    float target_x;
    float target_y;
    
    // Gate Friction & Stride State
    float gate_state;
    float stride_state; // The EMA state for the sweeping arc
    
    // 2nd-Order Physics State Variables
    float pos_lx, pos_ly;
    float vel_lx, vel_ly;
    
    bool was_active_l;
    float land_offset_l;
} Humanizer;

void humanizer_init(Humanizer* h);

void humanizer_process(Humanizer* h, int16_t* lx, int16_t* ly, int16_t* rx, int16_t* ry,
                       uint16_t circ_error, uint16_t smoothing_rate, uint16_t anti_deadzone, 
                       uint16_t walk_drift, uint16_t sprint_drift, 
                       uint16_t gate_slip, uint16_t landing_var, uint16_t passthrough);

#endif
