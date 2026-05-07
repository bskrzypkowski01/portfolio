#include "pll_sogi.h"
#include <math.h>
#include <stdbool.h>

/* =========================
   CONSTANTS
   ========================= */
#define TS            0.0001f       // 50 kHz
#define TWO_PI        6.283185307f

/* SOGI */
#define KSOGI         1.0f
#define SQRT2         1.41421356f

/* PLL PI (50 kHz) */
#define PLL_KP        20.0f
#define PLL_KI        3000.0f
//#define PLL_KP        8.0f
//#define PLL_KI        400.0f

/* Limits */
#define PI_INT_LIM    500.0f
#define OMEGA_MIN    200.0f           // ~32 Hz
#define OMEGA_MAX    500.0f           // ~80 Hz
#define U_MOD_MIN    50.0f            // anti-blowup near zero

/* =========================
   SOGI STEP (Euler Backward)
   ========================= */
static inline void SOGI_Step(SOGI_t *s, float vin, float omega)
{
    /*
      dv/dt = k*ω*(vin - v) - ω*q
      dq/dt = ω*v
    */

    float a = s->k * omega;
    float b = omega;

    float denom = 1.0f + TS * a + TS * TS * b * b;

    float v_new =
        ( s->v
        + TS * a * vin
        - TS * b * s->q ) / denom;

    float q_new =
        s->q + TS * b * v_new;

    s->v = v_new;
    s->q = q_new;
}

/* =========================
   INIT
   ========================= */
void PLL_SOGI_Init(PLL_SOGI_t *pll, float omega_nom)
{
    pll->sogi.v = 0.0f;
    pll->sogi.q = 0.0f;
    pll->sogi.k = KSOGI * SQRT2;

    pll->theta      = 0.0f;
    pll->omega      = omega_nom;
    pll->omega_init = omega_nom;

    pll->v_d = 0.0f;
    pll->v_q = 0.0f;

    pll->u_mod   = 0.0f;
    pll->pll_err = 0.0f;

    pll->pi_int = 0.0f;

    pll->omega_f   = omega_nom;
    pll->omega_sum = 0.0f;
    pll->omega_idx = 0;

    for (unsigned int i = 0; i < MA_LEN; i++)
    {
        pll->omega_buf[i] = omega_nom;
        pll->omega_sum   += omega_nom;
    }
}

/* =========================
   MAIN STEP (50 kHz)
   ========================= */
void PLL_SOGI_Step(PLL_SOGI_t *pll, float v_in)
{
    /* === 1. SOGI-QSG === */
    //SOGI_Step(&pll->sogi, v_in, pll->omega);
    /* === 1. SOGI-QSG (FIXED ω) === */
    SOGI_Step(&pll->sogi, v_in, pll->omega_init);

    /* === 2. Vector magnitude === */
    pll->u_mod = sqrtf(
        pll->sogi.v * pll->sogi.v +
        pll->sogi.q * pll->sogi.q);

    float u = pll->u_mod;
    if (u < U_MOD_MIN)
        u = U_MOD_MIN;

    /* === 3. Park transform === */
    float s = sinf(pll->theta);
    float c = cosf(pll->theta);

    pll->v_d =  pll->sogi.v * c + pll->sogi.q * s;
    pll->v_q = -pll->sogi.v * s + pll->sogi.q * c;

    /* === 4. Normalized PLL error === */
    pll->pll_err = pll->v_q / u;

    /* === 5. PI PLL (ANTI-WINDUP) === */
    float omega_pi =
        pll->omega_init
      + PLL_KP * pll->pll_err
      + pll->pi_int;

    bool saturated =
        (omega_pi > OMEGA_MAX) ||
        (omega_pi < OMEGA_MIN);

    if (!saturated)
    {
        pll->pi_int += PLL_KI * pll->pll_err * TS;

        if (pll->pi_int >  PI_INT_LIM) pll->pi_int =  PI_INT_LIM;
        if (pll->pi_int < -PI_INT_LIM) pll->pi_int = -PI_INT_LIM;
    }

    /* === 6. SOFT ω CLAMP === */
    if (omega_pi > OMEGA_MAX)
        pll->omega = OMEGA_MAX;
    else if (omega_pi < OMEGA_MIN)
        pll->omega = OMEGA_MIN;
    else
        pll->omega = omega_pi;

    /* === 7. Integrate angle === */
    pll->theta += pll->omega * TS;

    if (pll->theta >= TWO_PI) pll->theta -= TWO_PI;
    if (pll->theta <  0.0f)   pll->theta += TWO_PI;

    /* === 8. MA FILTER (OUTSIDE PLL LOOP) === */
    pll->omega_sum -= pll->omega_buf[pll->omega_idx];
    pll->omega_buf[pll->omega_idx] = pll->omega;
    pll->omega_sum += pll->omega;

    pll->omega_idx++;
    if (pll->omega_idx >= MA_LEN)
        pll->omega_idx = 0;

    pll->omega_f = pll->omega_sum / (float)MA_LEN;
}
