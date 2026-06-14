#include "humanizer.h"

// Q16.16 fixed point math - pure integer, no library needed
static inline int32_t fp_mul(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * b) >> 16);
}

static inline int32_t fp_div(int32_t a, int32_t b)
{
    if (b == 0) return 0;
    return (int32_t)(((int64_t)a << 16) / b);
}

static inline int32_t fp_from_int(int16_t a)
{
    return (int32_t)((int32_t)a << 16);
}

static inline int16_t fp_to_int(int32_t a)
{
    return (int16_t)(a >> 16);
}

static int32_t fp_next_rand(Humanizer* h)
{
    h->rng_state ^= h->rng_state << 13;
    h->rng_state ^= h->rng_state >> 17;
    h->rng_state ^= h->rng_state << 5;
    int16_t signed_val = (int16_t)(h->rng_state % 1000);
    return fp_div(fp_from_int(signed_val), fp_from_int(1000));
}

static const int32_t FIX_1    = 0x00010000;
static const int32_t FIX_NEG1 = 0xFFFF0000;
static const int32_t FIX_095  = 0x0000F333;

void humanizer_init(Humanizer* h)
{
    // Default settings
    h->settings.magnitude_cap        = 0x0000D999; // 0.85
    h->settings.drift_strength       = 0x00001470; // 0.08
    h->settings.drift_max            = 0x00004000; // 0.25
    h->settings.drift_retarget_frames = 400;
    h->settings.idle_threshold       = 0x00001999; // 0.10
    h->settings.enabled              = true;

    h->drift_lx = 0; h->drift_ly = 0;
    h->drift_rx = 0; h->drift_ry = 0;
    h->target_lx = 0; h->target_ly = 0;
    h->target_rx = 0; h->target_ry = 0;
    h->retarget_counter_l = 0;
    h->retarget_counter_r = 0;
    h->was_idle_l = true;
    h->was_idle_r = true;
    h->rng_state = 12345;
}

static void process_stick(
    Humanizer* h,
    int16_t* x, int16_t* y,
    int32_t* drift_x, int32_t* drift_y,
    int32_t* target_x, int32_t* target_y,
    uint32_t* retarget_counter,
    bool* was_idle)
{
    int32_t nx = fp_div(fp_from_int(*x), fp_from_int(32767));
    int32_t ny = fp_div(fp_from_int(*y), fp_from_int(32767));

    int32_t mag_sq = fp_mul(nx, nx) + fp_mul(ny, ny);
    int32_t idle_sq = fp_mul(h->settings.idle_threshold,
                             h->settings.idle_threshold);
    bool is_idle = (mag_sq < idle_sq);

    if (is_idle)
    {
        (*retarget_counter)++;
        if (*retarget_counter >= h->settings.drift_retarget_frames)
        {
            *retarget_counter = 0;
            *target_x = fp_mul(fp_next_rand(h), h->settings.drift_max);
            *target_y = fp_mul(fp_next_rand(h), h->settings.drift_max);
        }
        *drift_x = *drift_x + fp_mul(*target_x - *drift_x,
                                     h->settings.drift_strength);
        *drift_y = *drift_y + fp_mul(*target_y - *drift_y,
                                     h->settings.drift_strength);
        nx = nx + *drift_x;
        ny = ny + *drift_y;
        if (nx > FIX_1)    nx = FIX_1;
        if (nx < FIX_NEG1) nx = FIX_NEG1;
        if (ny > FIX_1)    ny = FIX_1;
        if (ny < FIX_NEG1) ny = FIX_NEG1;
    }
    else
    {
        *drift_x = fp_mul(*drift_x, FIX_095);
        *drift_y = fp_mul(*drift_y, FIX_095);
        *retarget_counter = 0;
    }

    *was_idle = is_idle;

    *x = fp_to_int(fp_mul(nx, fp_from_int(32767)));
    *y = fp_to_int(fp_mul(ny, fp_from_int(32767)));
}

void humanizer_process(Humanizer* h,
    int16_t* lx, int16_t* ly,
    int16_t* rx, int16_t* ry)
{
    if (!h->settings.enabled) return;

    process_stick(h, lx, ly,
        &h->drift_lx, &h->drift_ly,
        &h->target_lx, &h->target_ly,
        &h->retarget_counter_l,
        &h->was_idle_l);

    process_stick(h, rx, ry,
        &h->drift_rx, &h->drift_ry,
        &h->target_rx, &h->target_ry,
        &h->retarget_counter_r,
        &h->was_idle_r);
}
