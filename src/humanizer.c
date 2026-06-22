#include "humanizer.h"
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void humanizer_init(Humanizer* h) {
    h->wobble_phase = 0.0f;
    h->tilt_phase = 0.0f;
    h->gate_phase = 0.0f;
    
    h->ema_lx = 0.0f; h->ema_ly = 0.0f;
    h->ema_rx = 0.0f; h->ema_ry = 0.0f;
    
    h->was_active_l = false; h->land_offset_l = 0.0f;
    h->was_active_r = false; h->land_offset_r = 0.0f;
}

// Helper to process a single stick to avoid repeating code
static void process_stick(Humanizer* h, int16_t* axis_x, int16_t* axis_y, float* ema_x, float* ema_y, 
                          bool* was_active, float* land_offset,
                          uint16_t circ_error, uint16_t jitter_mag, uint16_t jitter_inner, uint16_t jitter_outer, 
                          uint16_t smoothing_rate, uint16_t gate_level, uint16_t tilt_deg, uint16_t landing_var) {
    
    // 1. Cartesian Smoothing (EMA) - Fixes the keyboard release "staircase"
    float alpha = 1.0f - (smoothing_rate * 0.009f); // 0 = Instant (1.0), 100 = Heavy smooth (0.1)
    if (alpha < 0.1f) alpha = 0.1f;
    
    *ema_x = (alpha * (float)(*axis_x)) + ((1.0f - alpha) * (*ema_x));
    *ema_y = (alpha * (float)(*axis_y)) + ((1.0f - alpha) * (*ema_y));
    
    float x = *ema_x / 32767.0f;
    float y = *ema_y / 32767.0f;

    float mag = sqrtf(x*x + y*y);
    float angle = atan2f(y, x);

    // Only apply complex math if the stick is actually being pushed
    if (mag > 0.01f) {
        float deflection = (mag > 1.0f) ? 1.0f : mag; // 0.0 to 1.0

        // 2. Landing Variation (Per-Press Dice Roll)
        if (landing_var > 0) {
            if (!(*was_active) && mag > 0.05f) { // Engagement threshold
                *land_offset = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f; // -1.0 to 1.0
                *was_active = true;
            }
            float land_deg_max = (landing_var / 100.0f) * 6.0f; 
            float land_rad = (*land_offset) * (land_deg_max * (float)M_PI / 180.0f);
            angle += land_rad; // Apply the crooked dice roll
        }
        
        // 3. Ergonomic Tilt (Wandering baseline)
        if (tilt_deg > 0) {
            float tilt_max = (tilt_deg / 100.0f) * 15.0f * (M_PI / 180.0f);
            angle += sinf(h->tilt_phase) * tilt_max;
        }

        // 4. Dynamic Wobble (The 3-Slider Curve)
        if (jitter_mag > 0) {
            float max_wobble = (jitter_mag / 100.0f) * (6.0f * (M_PI / 180.0f));
            float inner = jitter_inner / 100.0f;
            float outer = jitter_outer / 100.0f;
            
            // Linear interpolation curve based on stick deflection
            float curve = inner + (outer - inner) * deflection; 
            
            // Apply wobble scaled by the dynamic curve
            angle += sinf(h->wobble_phase) * max_wobble * curve;
        }

        // 5. Circularity Error (Hardware Calibration Flaw)
        // Applies a permanent, slightly square warp to the entire stick output
        if (circ_error > 0) {
            float circ = 1.0f + ((circ_error / 50.0f) * 0.15f * fabs(sinf(angle * 4.0f))); 
            mag = mag * circ;
        }

        // 6. Gate Slop (Outer-Ring Plastic Flex)
        // ONLY applies when the stick is pushed hard against the plastic edge (>80% deflection)
        if (gate_level > 0 && mag > 0.8f) { 
            // Fade it in smoothly between 80% and 100% so it doesn't abruptly snap
            float edge_fade = (mag - 0.8f) / 0.2f; 
            if (edge_fade > 1.0f) edge_fade = 1.0f;
            
            float slop = (gate_level / 100.0f) * 0.05f * sinf(h->gate_phase);
            mag += (slop * edge_fade);
        }

        // Convert back to cartesian
        x = cosf(angle) * mag;
        y = sinf(angle) * mag;
    } else {
        *was_active = false;
        *land_offset = 0.0f;
    }

    // Clamp and output
    if (x > 1.0f) x = 1.0f; if (x < -1.0f) x = -1.0f;
    if (y > 1.0f) y = 1.0f; if (y < -1.0f) y = -1.0f;

    *axis_x = (int16_t)(x * 32767.0f);
    *axis_y = (int16_t)(y * 32767.0f);
}

void humanizer_process(Humanizer* h, int16_t* lx, int16_t* ly, int16_t* rx, int16_t* ry,
                       uint16_t circ_error, 
                       uint16_t jitter_mag, uint16_t jitter_inner, uint16_t jitter_outer, 
                       uint16_t smoothing_rate, uint16_t gate_level,
                       uint16_t tilt_deg, uint16_t landing_var, uint16_t passthrough) {
    
    if (passthrough) return; // Killswitch

    // Advance oscillators
    h->wobble_phase += 0.4f; // Fast thumb shake
    h->tilt_phase   += 0.01f; // Slow breathing lean
    h->gate_phase   += 0.05f; // Medium plastic flex

    // Process Left Stick
    process_stick(h, lx, ly, &h->ema_lx, &h->ema_ly, &h->was_active_l, &h->land_offset_l,
                  circ_error, jitter_mag, jitter_inner, jitter_outer, 
                  smoothing_rate, gate_level, tilt_deg, landing_var);

    // Process Right Stick (optional depending on your target, but safe to run)
    process_stick(h, rx, ry, &h->ema_rx, &h->ema_ry, &h->was_active_r, &h->land_offset_r,
                  circ_error, jitter_mag, jitter_inner, jitter_outer, 
                  smoothing_rate, gate_level, tilt_deg, landing_var);
}
