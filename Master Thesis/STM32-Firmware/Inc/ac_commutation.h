#ifndef AC_COMMUTATION_H
#define AC_COMMUTATION_H

#include <stdint.h>

/* =========================
   AC HALF-CYCLE ENUM
   ========================= */
typedef enum
{
    AC_HALF_NEGATIVE = -1,
    AC_HALF_ZERO     =  0,
    AC_HALF_POSITIVE =  1
} AC_Half_t;

/* =========================
   AC COMMUTATION STRUCT
   ========================= */
typedef struct
{
    float dead_zone_rad;   /* Dead-zone around zero-cross [rad] */

    AC_Half_t half;        /* -1 / 0 / +1 */

    uint8_t allow_pwm;     /* F_ctrl */
    uint8_t F_pos;         /* positive half-cycle flag */
    uint8_t F_neg;         /* negative half-cycle flag */

} AC_Commutation_t;

/* =========================
   API
   ========================= */
void AC_Commutation_Init(AC_Commutation_t *c, float dead_zone_rad);
void AC_Commutation_Update(AC_Commutation_t *c, float theta);

#endif /* AC_COMMUTATION_H */
