/* plant_user_api.h */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double Ts, Th, Tc, mdot;
    double v_prev;
} Plant;

/* Exposed functions (become external only if compiled with -DUNIT_TEST) */
double parse_or(const char* s, double fallback);
double sat(double x, double lo, double hi);
double softabs(double x, double eps);
double safe_sq(double x, double cap);
double mu_water(double T_c);
double UA_func(double v_cmd, double Tstar);
double Psys(double t, double Ts);
void   plant_step(Plant* s, double omega_cmd_rpm, double v_cmd_rpm, double dt);

int16_t  pack_temp_q10(double T_c);
uint8_t  pack_v_prev_q10(double v_rpm);
uint8_t  pack_dt_ms(double dt);
uint16_t u16_from_le(uint8_t b0, uint8_t b1);
void     le_from_u16(uint8_t* b0, uint8_t* b1, uint16_t v);

#ifdef __cplusplus
}
#endif
