#include "humanizer.h"
#include <math.h>
#include <stdlib.h>

#define AXIS_MAX 32767.0f 

void humanizer_init(Humanizer* h) {
    h->current_noise_l = 0.0f;
    h->target_noise_l = 0.0f;
    h->prev_mag_l = 0.0f;
    
    h->current_noise_r = 0.0f;
    h->target_noise_r = 0.0f;
    h->prev_mag_r = 0.0f;
}

// Internal helper to apply math to a single stick
void process_stick(int16_t* out_x, int16_t* out_y, int16_t raw_x, int16_t raw_y, 
                   uint16_t error_pct, uint16_t deviation_level, 
                   float* current_noise, float* target_noise, float* prev_mag) {
    
    float x = (float)raw_x;
    float y = (float)raw_y;
    
    float magnitude = sqrtf((x * x) + (y * y));
    float mag_delta = magnitude - *prev_mag;
    *prev_mag = magnitude; 

    // --- 1. FLUID VELOCITY-AWARE ENGINE ---
    if (deviation_level > 0) {
        if (fabsf(*target_noise - *current_noise) < 0.05f) {
            *target_noise = ((float)rand() / (float)(RAND_MAX)) * 2.0f - 1.0f;
        }

        float sweep_speed = 0.03f; // Base speed for pressing/holding

        // If the key is releasing (magnitude is shrinking by at least 10 units to ignore static jitter)
        if (mag_delta < -10.0f) {
            // Dynamically scale the speed boost based on how fast the key is rising
            float speed_boost = fabsf(mag_delta) / 2000.0f;
            if (speed_boost > 0.4f) speed_boost = 0.4f; // Cap the max turbo at 0.4x
            sweep_speed += speed_boost;
        }

        *current_noise = (*current_noise) + ((*target_noise - *current_noise) * sweep_speed);
    } else {
        *current_noise = 0.0f;
        *target_noise = 0.0f;
    }

    // --- 2. INVERTED ANGULAR SCALING ---
    if (magnitude > 0) {
        float angle = atan2f(y, x);

        if (deviation_level > 0) {
            float max_wobble_rads = (deviation_level / 100.0f) * (6.0f * (M_PI / 180.0f));
            
            // Create a multiplier that is 1.0 at the edge, and grows up to 4.0 at the center
            float center_looseness = 1.0f - (magnitude / AXIS_MAX);
            if (center_looseness < 0.0f) center_looseness = 0.0f;
            float wobble_multiplier = 1.0f + (center_looseness * 3.0f);

            // Apply the amplified wobble to the angle
            angle += (*current_noise * max_wobble_rads * wobble_multiplier);
        }

        float limit = AXIS_MAX * (1.0f + (error_pct / 100.0f));
        if (magnitude > limit) {
            magnitude = limit; 
        }

        x = magnitude * cosf(angle);
        y = magnitude * sinf(angle);
    }

    if (x > AXIS_MAX) x = AXIS_MAX;
    if (x < -AXIS_MAX) x = -AXIS_MAX;
    if (y > AXIS_MAX) y = AXIS_MAX;
    if (y < -AXIS_MAX) y = -AXIS_MAX;

    *out_x = (int16_t)x;
    *out_y = (int16_t)y;
}

void humanizer_process(Humanizer* h, int16_t* lx, int16_t* ly, int16_t* rx, int16_t* ry, 
                       uint16_t circ_error, uint16_t axis_deviation) {
    
    process_stick(lx, ly, *lx, *ly, circ_error, axis_deviation, 
                  &(h->current_noise_l), &(h->target_noise_l), &(h->prev_mag_l));
                  
    process_stick(rx, ry, *rx, *ry, circ_error, axis_deviation, 
                  &(h->current_noise_r), &(h->target_noise_r), &(h->prev_mag_r));
}
