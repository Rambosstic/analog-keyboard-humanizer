#ifndef HUMANIZER_H
#define HUMANIZER_H

#include <stdint.h>

typedef struct {
    float phase_l;    // The continuous "time" clock for the left stick wave
    float prev_mag_l; // Tracks velocity for the release turbo
    
    float phase_r;    // The continuous "time" clock for the right stick wave
    float prev_mag_r; 
} Humanizer;

void humanizer_init(Humanizer* h);

void humanizer_process(Humanizer* h, int16_t* lx, int16_t* ly, int16_t* rx, int16_t* ry, 
                       uint16_t circ_error, uint16_t axis_deviation);

#endif
