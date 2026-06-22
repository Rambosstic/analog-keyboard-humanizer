#include "humanizer.h"
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef TWO_PI
#define TWO_PI (2.0f * (float)M_PI)
#endif

void humanizer_init(Humanizer* h) {
    h->wobble_phase = 0.0f;
    h->tilt_phase = 0.0f;
    h->gate_phase = 0.0f;
    
    h->pos_lx = 0.0f; h->pos_ly = 0.0f;
    h->vel_lx = 0.0f; h->vel_ly = 0.0f;
    h->pos_rx = 0.0f; h->pos_ry = 0.0f;
    h->vel_rx = 0.0f; h->vel_ry = 0.0f;
    
    h->was_active_l = false; h->land_offset_l = 0.0f;
    h->was_active_r = false; h->land_offset_r = 0.0f;
}

static void process_stick(Humanizer* h, int16_t* axis_x, int16_t* axis_y, 
                          float* pos_x, float* pos_y, float* vel_x, float* vel_y,
                          bool* was_active, float* land_offset,
                          uint16_t circ_error, uint16_t jitter_mag, uint16_t jitter_inner, uint16_t jitter_outer, 
                          uint16_t smoothing_rate, uint16_t gate_level, uint16_t tilt_deg, uint16_t landing_var, uint16_t diagonal_feel) {
    
    float tx = (float)(*axis_x);
    float ty = (float)(*axis_y);

    // 1. Axis Coupling (Diagonal Feel)
    if (diagonal_feel > 0) {
        float coupling = (diagonal_feel / 100.0f) * 0.30f; // Max 30% blend
        float cx = tx;
        float cy = ty;
        // The presence of Y pulls X further in X's current direction
        tx += coupling * cx * (fabsf(cy) / 32767.0f);
        ty += coupling * cy * (fabsf(cx) / 32767.0f);
    }

    // 2. 2nd-Order Physics Smoothing (Inertia)
    if (smoothing_rate == 0) {
        // Zero lag, snap directly
        *pos_x = tx;
        *pos_y = ty;
        *vel_x = 0.0f;
        *vel_y = 0.0f;
    } else {
        // Map slider (1-100) to spring frequency in Hz (25Hz to 3Hz)
        float freq_hz = 25.0f - (smoothing_rate * 0.22f); 
        if (freq_hz < 3.0f) freq_hz = 3.0f; // Hard floor to prevent infinite float
        
        float w = TWO_PI * freq_hz;
        float k = w * w;      // Spring stiffness
        float c = 2.0f * w;   // Critical damping
        float dt = 0.004f;    // 250Hz loop duration
        
        float force_x = k * (tx - *pos_x) - c * (*vel_x);
        float force_y = k * (ty - *pos_y) - c * (*vel_y);
        
        *vel_x += force_x * dt;
        *vel_y += force_y * dt;
        *pos_x += (*vel_x) * dt;
        *pos_y += (*vel_y) * dt;
    }
    
    float x = *pos_x / 32767.0f;
    float y = *pos_y / 32767.0f;

    float mag = sqrtf(x*x + y*y);
    float angle = atan2f(y, x);

    // Only apply complex math if the stick is actually being pushed
    if (mag > 0.01f) {
        
        // The Virtual Plastic Ring (Gate Clamp)
        if (mag > 1.0f) {
            mag = 1.0f; 
        }

        float deflection = mag; // Safely clamped to a 0.0 - 1.0 scale

        // 3. Landing Variation (Per-Press Dice Roll)
        if (landing_var > 0) {
            if (!(*was_active) && mag > 0.05f) { 
                *land_offset = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f; 
                *was_active = true;
            }
            float land_deg_max = (landing_var / 100.0f) * 6.0f; 
            float land_rad = (*land_offset) * (land_deg_max * (float)M_PI / 180.0f);
            angle += land_rad; 
        }
        
        // 4. Ergonomic Tilt (Wandering baseline)
        // Blooming effect: scales intensity based on current physical stick deflection
        if (tilt_deg > 0) {
            float tilt_max = (tilt_deg / 100.0f) * 15.0f * (M_PI / 180.0f);
            angle += sinf(h->tilt_phase) * tilt_max * deflection;
        }

        // 5. Dynamic Wobble (The 3-Slider Curve)
        if (jitter_mag > 0) {
            float max_wobble = (jitter_mag / 100.0f) * (6.0f * (M_PI / 180.0f));
            float inner = jitter_inner / 100.0f;
            float outer = jitter_outer / 100.0f;
            
            float curve = inner + (outer - inner) * deflection; 
            angle += sinf(h->wobble_phase) * max_wobble * curve;
        }

        // 6. Circularity Error (Hardware Calibration Flaw)
        if (circ_error > 0) {
            float circ = 1.0f + ((circ_error / 50.0f) * 0.414f * fabsf(sinf(angle * 2.0f))); 
            mag = mag * circ;
        }

        // 7. Gate Slop (Outer-Ring Plastic Flex)
        if (gate_level > 0 && mag > 0.8f) { 
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

    // Clamp and output (Safety net for integer conversion)
    if (x > 1.0f) x = 1.0f; if (x < -1.0f) x = -1.0f;
    if (y > 1.0f) y = 1.0f; if (y < -1.0f) y = -1.0f;

    *axis_x = (int16_t)(x * 32767.0f);
    *axis_y = (int16_t)(y * 32767.0f);
}

void humanizer_process(Humanizer* h, int16_t* lx, int16_t* ly, int16_t* rx, int16_t* ry,
                       uint16_t circ_error, 
                       uint16_t jitter_mag, uint16_t jitter_inner, uint16_t jitter_outer, 
                       uint16_t smoothing_rate, uint16_t gate_level,
                       uint16_t tilt_deg, uint16_t landing_var, uint16_t diagonal_feel, uint16_t passthrough) {
    
    if (passthrough) return; 

    // Advance oscillators and wrap them to prevent float precision death
    h->wobble_phase += 0.4f; 
    if (h->wobble_phase > TWO_PI) h->wobble_phase -= TWO_PI;

    h->tilt_phase   += 0.01f; 
    if (h->tilt_phase > TWO_PI) h->tilt_phase -= TWO_PI;

    h->gate_phase   += 0.05f; 
    if (h->gate_phase > TWO_PI) h->gate_phase -= TWO_PI;

    // Process Left Stick
    process_stick(h, lx, ly, &h->pos_lx, &h->pos_ly, &h->vel_lx, &h->vel_ly, 
                  &h->was_active_l, &h->land_offset_l,
                  circ_error, jitter_mag, jitter_inner, jitter_outer, 
                  smoothing_rate, gate_level, tilt_deg, landing_var, diagonal_feel);

    // Process Right Stick 
    process_stick(h, rx, ry, &h->pos_rx, &h->pos_ry, &h->vel_rx, &h->vel_ry, 
                  &h->was_active_r, &h->land_offset_r,
                  circ_error, jitter_mag, jitter_inner, jitter_outer, 
                  smoothing_rate, gate_level, tilt_deg, landing_var, diagonal_feel);
}
