// ctrl_set.c — One-shot: Ts_sp (0x301) + gains (0x300, 0x302)
// Build:  gcc -O2 -Wall -o ctrl_set ctrl_set.c -lm
// Usage:  ./ctrl_set <ifname> <Ts_sp_C> [--kp KpT] [--ki KiT] [--kd KdT] [--kaw kawT]
//                                         [--kpm Kpm] [--kim Kim] [--kawm kawm] [--kvw kvw] [--kwv kwv]
//         Add --no-params to send only 0x301.
// Example:
//   ./ctrl_set vcan0 30.0 --kp 120 --ki 0.15 --kd 5 --kaw 4 --kpm 150 --kim 0.02 --kawm 8 --kvw -0.1 --kwv -0.03

#define _GNU_SOURCE
#include <errno.h>
#include <math.h>
#include <net/if.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/* ---------- Test exposure macros (no impact in production) ---------- */
#ifdef UNIT_TEST
  #define EXPOSE
#else
  #define EXPOSE static
#endif

/* ---------- Utilities / Packing ---------- */
EXPOSE void die(const char* m){ perror(m); exit(EXIT_FAILURE); }

EXPOSE void bind_socket(int s, const char* ifname){
    struct ifreq ifr;
    struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) die("SIOCGIFINDEX");
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) die("bind");
}

EXPOSE void u16_to_le(uint8_t* b, uint16_t v){ b[0] = (uint8_t)(v & 0xFFu); b[1] = (uint8_t)((v >> 8) & 0xFFu); }

/* Quantizers */
EXPOSE int16_t  to_q01(float x){ long v = lroundf(x * 10.0f);  if (v < -32768) v = -32768; if (v >  32767) v =  32767; return (int16_t)v; }
EXPOSE uint16_t to_q88(float x){ long v = lroundf(x * 256.0f); if (v < 0)      v = 0;      if (v > 65535) v = 65535; return (uint16_t)v; }
EXPOSE uint8_t  to_q44(float x){ long v = lroundf(x * 16.0f);  if (v < 0)      v = 0;      if (v >   255) v =   255; return (uint8_t)v; }

/* Small helper to send a CAN frame (keeps main tiny) */
static void send_frame_or_die(int s, const struct can_frame* f, const char* tag){
    ssize_t n = send(s, f, sizeof(*f), 0);
    if (n != (ssize_t)sizeof(*f)) die(tag);
}

/* ---------- Parameter bundle ---------- */
typedef struct {
    float Ts_sp_C;
    float KpT, KiT, KdT, kawT;
    float Kpm, Kim, kawm, kvw, kwv;
    bool  send_params;
} CtrlParams;

/* Defaults (match your original controller defaults) */
static CtrlParams default_params(float Ts_sp){
    CtrlParams p = {
        .Ts_sp_C = Ts_sp,
        .KpT = 100.6f, .KiT = 0.10f, .KdT = 4.0f,  .kawT = 5.0f,
        .Kpm = 130.0f, .Kim = 0.01f, .kawm = 10.0f, .kvw = -0.15f, .kwv = -0.02f,
        .send_params = true
    };
    return p;
}

/* Minimal argument parser (returns false on bad arg) */
static bool parse_args(int argc, char** argv, CtrlParams* out){
    if (argc < 3) return false;
    out->Ts_sp_C = strtof(argv[2], NULL);
    CtrlParams def = default_params(out->Ts_sp_C);
    *out = def;

    for (int i = 3; i < argc; i++){
        const char* a = argv[i];
        if (!strcmp(a, "--no-params")) { out->send_params = false; continue; }
        #define NEXT_FLOAT(VAR) do{ if (i+1 >= argc) return false; (VAR) = strtof(argv[++i], NULL); }while(0)

        if      (!strcmp(a, "--kp"))  NEXT_FLOAT(out->KpT);
        else if (!strcmp(a, "--ki"))  NEXT_FLOAT(out->KiT);
        else if (!strcmp(a, "--kd"))  NEXT_FLOAT(out->KdT);
        else if (!strcmp(a, "--kaw")) NEXT_FLOAT(out->kawT);
        else if (!strcmp(a, "--kpm")) NEXT_FLOAT(out->Kpm);
        else if (!strcmp(a, "--kim")) NEXT_FLOAT(out->Kim);
        else if (!strcmp(a, "--kawm"))NEXT_FLOAT(out->kawm);
        else if (!strcmp(a, "--kvw")) NEXT_FLOAT(out->kvw);
        else if (!strcmp(a, "--kwv")) NEXT_FLOAT(out->kwv);
        else return false;
        #undef NEXT_FLOAT
    }
    return true;
}

/* ---------- Frame builders (also used in tests) ---------- */
EXPOSE void build_setpoint_frame(float Ts_sp_C, struct can_frame* sp){
    memset(sp, 0, sizeof(*sp));
    sp->can_id = 0x301; sp->len = 8;
    u16_to_le(&sp->data[0], (uint16_t)to_q01(Ts_sp_C));
}

EXPOSE void build_params_frames(const CtrlParams* p, struct can_frame* p1, struct can_frame* p2){
    /* 0x300: KpT/KiT/KdT (q8.8), kawT (q4.4) */
    memset(p1, 0, sizeof(*p1));
    p1->can_id = 0x300; p1->len = 8;
    u16_to_le(&p1->data[0], to_q88(p->KpT));
    u16_to_le(&p1->data[2], to_q88(p->KiT));
    u16_to_le(&p1->data[4], to_q88(p->KdT));
    p1->data[6] = to_q44(p->kawT);

    /* 0x302: Kpm/Kim (q8.8), kawm/kvw/kwv (q4.4) */
    memset(p2, 0, sizeof(*p2));
    p2->can_id = 0x302; p2->len = 8;
    u16_to_le(&p2->data[0], to_q88(p->Kpm));
    u16_to_le(&p2->data[2], to_q88(p->Kim));
    p2->data[4] = to_q44(p->kawm);
    p2->data[5] = to_q44(p->kvw);
    p2->data[6] = to_q44(p->kwv);
}

/* Test-only consolidated builder to avoid sockets in gtests */
#ifdef UNIT_TEST
EXPOSE void build_ctrl_frames(const CtrlParams* p,
                              struct can_frame* sp,
                              struct can_frame* f300,
                              struct can_frame* f302)
{

    
    if (sp)   build_setpoint_frame(p->Ts_sp_C, sp);
    if (p->send_params) {
        if (f300 && f302) build_params_frames(p, f300, f302);
    } else {
        if (f300) memset(f300, 0, sizeof(*f300));
        if (f302) memset(f302, 0, sizeof(*f302));
    }
}
#endif

/* ---------- Usage message ---------- */
static void usage(const char* prog){
    fprintf(stderr,
        "Usage: %s <ifname> <Ts_sp_C> "
        "[--kp KpT] [--ki KiT] [--kd KdT] [--kaw kawT] "
        "[--kpm Kpm] [--kim Kim] [--kawm kawm] [--kvw kvw] [--kwv kwv] "
        "[--no-params]\n", prog);
}

/* ---------- Main (excluded in unit tests) ---------- */
#ifndef UNIT_TEST
int main(int argc, char** argv){
    if (argc < 3){ usage(argv[0]); return 1; }

    const char* ifname = argv[1];
    CtrlParams P;
    if (!parse_args(argc, argv, &P)){ usage(argv[0]); return 1; }

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) die("socket");
    bind_socket(s, ifname);

    /* 0x301: setpoint */
    struct can_frame sp;
    build_setpoint_frame(P.Ts_sp_C, &sp);
    send_frame_or_die(s, &sp, "send 0x301");
    printf("[A] 0x301 Ts_sp=%.1f°C\n", P.Ts_sp_C);

    if (P.send_params){
        struct can_frame f300, f302;
        build_params_frames(&P, &f300, &f302);
        send_frame_or_die(s, &f300, "send 0x300");
        printf("[A] 0x300 KpT=%.3g KiT=%.3g KdT=%.3g kawT=%.3g\n", P.KpT, P.KiT, P.KdT, P.kawT);
        send_frame_or_die(s, &f302, "send 0x302");
        printf("[A] 0x302 Kpm=%.3g Kim=%.3g kawm=%.3g kvw=%.3g kwv=%.3g\n",
               P.Kpm, P.Kim, P.kawm, P.kvw, P.kwv);
    }

    close(s);
    return 0;
}
#endif
