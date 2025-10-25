#pragma once
#include <stdbool.h>
#include <linux/can.h>
typedef struct {
    float Ts_sp_C;
    float KpT, KiT, KdT, kawT;
    float Kpm, Kim, kawm, kvw, kwv;
    bool  send_params;
} CtrlParams;

#ifdef __cplusplus
extern "C" {
#endif

void     bind_socket(int s, const char* ifname);
void     u16_to_le(unsigned char* b, unsigned short v);
short    to_q01(float x);
unsigned short to_q88(float x);
unsigned char  to_q44(float x);

void build_ctrl_frames(const CtrlParams* p,
                       struct can_frame* sp,
                       struct can_frame* p1,
                       struct can_frame* p2);

#ifdef __cplusplus
}
#endif
