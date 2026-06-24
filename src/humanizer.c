#include "humanizer.h"
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef TWO_PI
#define TWO_PI (2.0f * (float)M_PI)
#endif

static float clamp_abs(float val, float max_val) {
    if (val > max_val) return max_val;
    if (val < -max_val) return -max_val;
    return val;
}

void humanizer_init(Humanizer* h) {
    h->tremor_state = 0.0f; h->tilt_state = 0.0f; h->gate_state = 0.0f;
    h->pos_lx = 0.0f; h->pos_ly = 0.0f;
    h->vel_lx = 0.0f; h->vel_ly = 0.0f;
    h->was_active_l = false; h->land_offset_l = 0.0f;
}

static void process_left_stick(Humanizer* h, int16_t* axis_x, int16_t* axis_y, 
                          uint16_t circ_error, uint16_t jitter_mag, uint16_t jitter_inner, uint16_t jitter_outer, 
                          uint16_t smoothing_rate, uint16_t gate_level, uint16_t variance_level, int16_t ergo_tilt, 
                          uint16_t landing_var, uint16_t diagonal_feel, uint16_t anti_deadzone) {
    
    // --- BACKGROUND STOCHASTIC PINK NOISE (Boosted for actual output) ---
    float noise_tremor = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
    float noise_tilt   = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
    float noise_gate   = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;

    h->tremor_state = clamp_abs((h->tremor_state * 0.90f) + (noise_tremor * 0.25f), 1.0f); 
    h->tilt_state   = clamp_abs((h->tilt_state * 0.995f) + (noise_tilt * 0.05f), 1.0f);   
    h->gate_state   = clamp_abs((h->gate_state * 0.95f) + (noise_gate * 0.15f), 1.0f); 

    // Normalize input
    float tx = (float)(*axis_x) / 32767.0f;
    float ty = (float)(*axis_y) / 32767.0f;

    // 1. Anti-Deadzone
    float raw_mag_initial = sqrtf(tx*tx + ty*ty);
    if (anti_deadzone > 0 && raw_mag_initial > 0.01f) {
        float ad_floor = anti_deadzone / 100.0f;
        float scaled_mag = ad_floor + raw_mag_initial * (1.0f - ad_floor);
        tx = (tx / raw_mag_initial) * scaled_mag;
        ty = (ty / raw_mag_initial) * scaled_mag;
    }

    // 2. Elliptical Grid Mapping
    if (circ_error < 50) {
        float circle_tx = tx * sqrtf(1.0f - (ty * ty) / 2.0f);
        float circle_ty = ty * sqrtf(1.0f - (tx * tx) / 2.0f);
        float blend = circ_error / 50.0f;
        tx = circle_tx + (tx - circle_tx) * blend;
        ty = circle_ty + (ty - circle_ty) * blend;
    }

    // 3. Axis Coupling
    if (diagonal_feel > 0) {
        float blend = (diagonal_feel / 100.0f) * 0.30f; 
        float abs_x = fabsf(tx);
        float abs_y = fabsf(ty);
        if (abs_x > abs_y && abs_y > 0.01f) {
            ty += (tx > 0.0f ? abs_y : -abs_y) * blend; 
        } else if (abs_y > abs_x && abs_x > 0.01f) {
            tx += (ty > 0.0f ? abs_x : -abs_x) * blend;
        }
    }

    float target_mag = sqrtf(tx*tx + ty*ty);

    // 4. Inertia Smoothing
    if (smoothing_rate == 0) {
        h->pos_lx = tx; h->pos_ly = ty;
        h->vel_lx = 0.0f; h->vel_ly = 0.0f;
    } else {
        float freq_hz = 25.0f - (smoothing_rate * 0.22f); 
        if (freq_hz < 3.0f) freq_hz = 3.0f; 
        float w = TWO_PI * freq_hz;
        float k = w * w;      
        float c = 2.0f * w;   
        float dt = 0.004f;    
        
        float force_x = k * (tx - h->pos_lx) - c * (h->vel_lx);
        float force_y = k * (ty - h->pos_ly) - c * (h->vel_ly);
        
        h->vel_lx += force_x * dt; h->vel_ly += force_y * dt;
        h->pos_lx += (h->vel_lx) * dt; h->pos_ly += (h->vel_ly) * dt;
    }
    
    // 5. Magnitude Recovery
    float spring_mag = sqrtf((h->pos_lx)*(h->pos_lx) + (h->pos_ly)*(h->pos_ly));
    if (target_mag > 0.95f && spring_mag < 0.95f && spring_mag > 0.1f) {
        float correction = 1.0f + (0.95f - spring_mag) * 0.3f; 
        h->pos_lx *= correction;
        h->pos_ly *= correction;
    }

    float x = h->pos_lx;
    float y = h->pos_ly;
    float mag = sqrtf(x*x + y*y);
    float angle = atan2f(y, x);

    if (mag > 0.01f) {
        // --- FIXED: Circle to Square Boundary ---
        float square_max = 1.0f / fmaxf(fabsf(cosf(angle)), fabsf(sinf(angle)));
        float allowed_max = 1.0f + (square_max - 1.0f) * (circ_error / 50.0f);
        if (mag > allowed_max) mag = allowed_max;
        
        float deflection = mag > 1.0f ? 1.0f : mag; // Keep at 1.0 for wobble math

        // 6. Landing Variation
        if (landing_var > 0) {
            if (!(h->was_active_l) && mag > 0.05f) { 
                h->land_offset_l = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f; 
                h->was_active_l = true;
            }
            float land_deg_max = (landing_var / 100.0f) * 6.0f; 
            float land_rad = (h->land_offset_l) * (land_deg_max * (float)M_PI / 180.0f);
            angle += land_rad; 
        }
        
        // 7. Steady Stick Deflection Variance
        if (variance_level > 0) {
            float variance_max = (variance_level / 100.0f) * 15.0f * (M_PI / 180.0f);
            angle += (h->tilt_state) * variance_max * deflection;
        }

        // 8. Dynamic Wobble
        if (jitter_mag > 0) {
            float max_wobble = (jitter_mag / 100.0f) * (6.0f * (M_PI / 180.0f));
            float inner = jitter_inner / 100.0f;
            float outer = jitter_outer / 100.0f;
            float curve = inner + (outer - inner) * deflection; 
            angle += (h->tremor_state) * max_wobble * curve;
        }

        // 9. Gate Slop
        if (gate_level > 0 && mag > 0.8f) { 
            float edge_fade = (mag - 0.8f) / 0.2f; 
            if (edge_fade > 1.0f) edge_fade = 1.0f;
            float slop = (gate_level / 100.0f) * 0.05f * (h->gate_state);
            mag += (slop * edge_fade);
        }

        // 10. Ergonomic Tilt (True Static Rotation)
        if (ergo_tilt != 0) {
            angle += ergo_tilt * (M_PI / 180.0f);
        }

        x = cosf(angle) * mag;
        y = sinf(angle) * mag;
    } else {
        h->was_active_l = false;
        h->land_offset_l = 0.0f;
    }

    x = clamp_abs(x, 1.0f);
    y = clamp_abs(y, 1.0f);

    *axis_x = (int16_t)(x * 32767.0f);
    *axis_y = (int16_t)(y * 32767.0f);
}

void humanizer_process(Humanizer* h, int16_t* lx, int16_t* ly, int16_t* rx, int16_t* ry,
                       uint16_t circ_error, uint16_t jitter_mag, uint16_t jitter_inner, uint16_t jitter_outer, 
                       uint16_t smoothing_rate, uint16_t gate_level, uint16_t variance_level, int16_t ergo_tilt, uint16_t landing_var, 
                       uint16_t diagonal_feel, uint16_t anti_deadzone, uint16_t passthrough) {
    
    if (passthrough) return; 

    // We ONLY process the left stick. Right stick (Mouse Aim) bypasses the CPU completely.
    process_left_stick(h, lx, ly, 
                       circ_error, jitter_mag, jitter_inner, jitter_outer, 
                       smoothing_rate, gate_level, variance_level, ergo_tilt, landing_var, 
                       diagonal_feel, anti_deadzone);
}
