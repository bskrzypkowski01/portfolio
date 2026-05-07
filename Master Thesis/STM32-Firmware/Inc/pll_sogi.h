#ifndef PLL_SOGI_H
#define PLL_SOGI_H

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
   CONFIG
   ========================= */
#define MA_LEN 200          // 2 ms @ 100 kHz

/* =========================
   SOGI-QSG
   ========================= */
typedef struct
{
  float v;      // u'  (alpha)
  float q;      // qu' (beta)
  float k;      // Ksogi * sqrt(2)
} SOGI_t;

/* =========================
   PLL-SOGI
   ========================= */
typedef struct
{
  /* --- SOGI --- */
  SOGI_t sogi;

  /* --- PLL --- */
  float theta;        // phase angle [rad]
  float omega;        // ω (used by SOGI + integrator)
  float omega_init;   // ω_INIT

  float v_d;
  float v_q;

  /* --- Normalization --- */
  float u_mod;
  float pll_err;

  /* --- PI --- */
  float pi_int;

  /* --- MA filter (diagnostic) --- */
  float omega_f;
  float omega_buf[MA_LEN];
  float omega_sum;
  unsigned int omega_idx;

} PLL_SOGI_t;

/* =========================
   API
   ========================= */
void PLL_SOGI_Init(PLL_SOGI_t *pll, float omega_nom);
void PLL_SOGI_Step(PLL_SOGI_t *pll, float v_in);

#ifdef __cplusplus
}
#endif

#endif /* PLL_SOGI_H */
