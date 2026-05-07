#include "ac_commutation.h"
#include <math.h>

#define TWO_PI  (6.283185307179586f)
#define PI      (3.141592653589793f)

void AC_Commutation_Init(AC_Commutation_t *c, float dead_zone_rad)
{
    c->dead_zone_rad = dead_zone_rad;

    c->half      = AC_HALF_ZERO;
    c->allow_pwm = 0;
    c->F_pos     = 0;
    c->F_neg     = 0;
}

void AC_Commutation_Update(AC_Commutation_t *c, float theta)
{
    /* Normalize angle to <0, 2π) */
    float th = fmodf(theta, TWO_PI);
    if (th < 0.0f)
        th += TWO_PI;

    /* Distance to zero crossings */
    float d0 = th;
    float d1 = fabsf(th - PI);
    float d2 = fabsf(th - TWO_PI);

    float dist_to_zero = fminf(d0, fminf(d1, d2));

    /* ===== DEAD ZONE ===== */
    if (dist_to_zero < c->dead_zone_rad)
    {
        c->half      = AC_HALF_ZERO;
        c->allow_pwm = 0;
        c->F_pos     = 0;
        c->F_neg     = 0;
        return;
    }

    /* ===== NORMAL OPERATION ===== */
    c->allow_pwm = 1;

    if (th < PI)
    {
        c->half  = AC_HALF_POSITIVE;
        c->F_pos = 1;
        c->F_neg = 0;
    }
    else
    {
        c->half  = AC_HALF_NEGATIVE;
        c->F_pos = 0;
        c->F_neg = 1;
    }
}
