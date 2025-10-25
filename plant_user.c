// plant_user.c — Plant on C: RX (0x201) omega_cmd,v_cmd; integrate plant; TX (0x202) Ts,Th,Tc,v_prev,dt
// Build:  gcc -O2 -Wall -o plant_user plant_user.c -lm
// Run:    ./plant_user vcan0 --Ts 60 --Th 40 --Tc 20 --v_prev 1200 --dt_ms 15 --mdot 0.25

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#ifdef UNIT_TEST
  #define EXPOSE /* external linkage in tests */
#else
  #define EXPOSE static
#endif

static void die(const char* m){ perror(m); exit(EXIT_FAILURE); }
EXPOSE  double parse_or(const char* s, double fallback){
    if (!s || !*s) return fallback;          // null or empty string → use fallback
    char* end = NULL;
    double v = strtod(s, &end);              // convert string to double
    if (end && *end == '\0') return v;       // valid numeric string → return parsed value
    return fallback;                         // otherwise → fallback
}

static void bind_socket(int s, const char* ifname){
    struct ifreq ifr; struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) die("SIOCGIFINDEX");
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) die("bind");
}

static  uint64_t now_ms(void){
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000ULL + ts.tv_nsec/1000000ULL;
}
EXPOSE  double sat(double x, double lo, double hi){
    return (x < lo) ? lo : (x > hi ? hi : x);
}
EXPOSE  double softabs(double x, double eps){ // like Python softabs
    return sqrt(x*x + eps*eps);
}
EXPOSE  double safe_sq(double x, double cap){
    if (x >  cap) x =  cap;
    if (x < -cap) x = -cap;
    return x*x;
}

/*** -------- Parameters (mirroring your Python) -------- ***/
// Thermo (fluid)
static const double cp   = 4180.0;   // J/(kg·K)
static const double Ch   = 1.5e4;    // J/K
static const double Cr   = 1.0e4;    // J/K
static const double T_amb= 25.0;     // °C

// System node
static const double Cs   = 3.0e3;    // J/K  (your Cs=3e3 in this snippet)
static const double Gsh  = 30.0;     // W/K

// Hydraulics
static const double Lh   = 2.0e6;    // Pa·s^2/kg
static const double a0   = 6894.76 * 0.00011066669385127739; // Pa/(rad/s)^2
static const double b    = 6894.76 * 1.659117628724065;      // Pa·s^2/kg^2
static const double Rh0  = 1.5e7;    // Pa·s^2/kg^2

// Radiator UA (fan after pump)
static const double UA0  = 120.0;
static const double kf   = 60.0;
static const double nexp = 0.65;

// Limits
static const double Ts_min=-400.0, Ts_max=1500.0;
static const double Th_min=-400.0, Th_max=1300.0;
static const double Tc_min=-400.0, Tc_max=1300.0;
static const double mdot_min=0.0,  mdot_max=1500.0;
static const double omega_max=4000.0, v_max=2800.0;

// Soft-abs / square caps
static const double SQR_CAP_OMEGA = 20000.0;

/*** -------- Models -------- ***/
EXPOSE double mu_water(double T_c){ // Pa·s, viscosity vs temperature (like your Python mu())
    double T_c_clip = sat(T_c, -10.0, 120.0);
    double T_K = T_c_clip + 273.15;
    const double A=2.414e-5, B=247.8, C=140.0;
    return A * exp(B / (T_K - C));
}
EXPOSE double UA_func(double v_cmd, double Tstar){
    (void)Tstar; // not used in UA but keep signature for parity
    double v_eff = sat(v_cmd, 0.0, 600.0);
    double ua = UA0 + kf * pow(fmax(v_eff,0.0), nexp);
    if (ua < 1.0) ua = 1.0;
    if (ua > 5e3) ua = 5e3;
    return ua;
}
EXPOSE double Psys(double t, double Ts){
    (void)t;
    double base = 180.0;            // W
    double alpha = 0.002;           // per K
    double p = base * (1.0 + alpha*(Ts - 60.0));
    if (p < 0.0) p = 0.0;
    if (p > 2e5) p = 2e5;
    return p;
}

/*** -------- Plant state and RHS -------- ***/
typedef struct {
    double Ts, Th, Tc, mdot;
    double v_prev; // last applied v_cmd (for logging/feedback)
} Plant;

static void plant_rhs(const Plant* s, double omega_cmd_rpm, double v_cmd_rpm,
                      double *dTs, double *dTh, double *dTc, double *dmdot)
{
    // Convert omega rpm -> rad/s-equivalent for pump law: we used omega (rad/s) in Python safe_square
    // In your Python, a0 multiplies omega_cmd^2 where omega_cmd looked like "rpm" numbers;
    // to match behavior, we keep units consistent with your original use (treat rpm as an abstract speed).
    double Ts = sat(s->Ts, Ts_min, Ts_max);
    double Th = sat(s->Th, Th_min, Th_max);
    double Tc = sat(s->Tc, Tc_min, Tc_max);
    double mdot = sat(s->mdot, mdot_min, mdot_max);

    double Tstar = sat(0.5*(Th + Tc), Tc_min, Th_max);

    // System -> coolant conduction
    double q_sh   = Gsh * (Ts - Th);                 // W
    double q_conv = mdot * cp * (Th - Tc);          // W

    // System node
    double dTs_loc = (Psys(0.0, Ts) - q_sh) / Cs;

    // Fluid nodes
    double dTh_loc = ( q_sh - q_conv ) / Ch;
    double dTc_loc = ( q_conv - UA_func(v_cmd_rpm, Tstar)*(Tc - T_amb) ) / Cr;

    // Hydraulics
    double dP_pump = a0 * safe_sq(omega_cmd_rpm, SQR_CAP_OMEGA) - b * safe_sq(mdot, 10.0);
    double Rh = Rh0 * (mu_water(Tstar)/mu_water(60.0));
    double dP_loss = Rh * mdot * softabs(mdot, 1e-9);
    double dmdot_loc = (dP_pump - dP_loss) / Lh;

    // Guard derivatives
    *dTs = sat(dTs_loc, -500.0,  500.0);
    *dTh = sat(dTh_loc, -500.0,  500.0);
    *dTc = sat(dTc_loc, -500.0,  500.0);
    *dmdot= sat(dmdot_loc, -500.0,  50.0);
}

EXPOSE void plant_step(Plant* s, double omega_cmd_rpm, double v_cmd_rpm, double dt){
    // RK2 (Heun)
    double dTs1,dTh1,dTc1,dmd1;
    plant_rhs(s, omega_cmd_rpm, v_cmd_rpm, &dTs1,&dTh1,&dTc1,&dmd1);

    Plant p = *s;
    p.Ts += dTs1*dt; p.Th += dTh1*dt; p.Tc += dTc1*dt; p.mdot += dmd1*dt;

    double dTs2,dTh2,dTc2,dmd2;
    plant_rhs(&p, omega_cmd_rpm, v_cmd_rpm, &dTs2,&dTh2,&dTc2,&dmd2);

    s->Ts   += 0.5*(dTs1 + dTs2)*dt;
    s->Th   += 0.5*(dTh1 + dTh2)*dt;
    s->Tc   += 0.5*(dTc1 + dTc2)*dt;
    s->mdot += 0.5*(dmd1 + dmd2)*dt;

    // clamp states
    s->Ts = sat(s->Ts, Ts_min, Ts_max);
    s->Th = sat(s->Th, Th_min, Th_max);
    s->Tc = sat(s->Tc, Tc_min, Tc_max);
    s->mdot = sat(s->mdot, mdot_min, mdot_max);
    s->v_prev = sat(v_cmd_rpm, 0.0, v_max);
}

/*** -------- Packing helpers -------- ***/
EXPOSE  int16_t pack_temp_q10(double T_c){
    // 0.1°C per LSB
    double q = round(T_c * 10.0);
    if (q < -32768.0) q = -32768.0;
    if (q >  32767.0) q =  32767.0;
    return (int16_t)q;
}

EXPOSE  uint8_t pack_v_prev_q10(double v_rpm){
    double q = floor(sat(v_rpm, 0.0, 2550.0) / 10.0 + 0.5);
    if (q < 0.0) { q = 0.0; }
    if (q > 255.0) { q = 255.0; }
    return (uint8_t)q;
}
EXPOSE  uint8_t pack_dt_ms(double dt){
    double ms = dt*1000.0;
    if (ms < 1.0) ms = 1.0;
    if (ms > 255.0) ms = 255.0;
    return (uint8_t)lround(ms);
}

EXPOSE  void le_from_u16(uint8_t* b0, uint8_t* b1, uint16_t v){
    *b0 = (uint8_t)(v & 0xFF);
    *b1 = (uint8_t)((v >> 8) & 0xFF);
}



/*** -------- Main -------- ***/
#ifndef UNIT_TEST
int main(int argc, char** argv){
    if (argc < 2){
        fprintf(stderr,
            "Usage: %s <ifname> [Ts] [Th] [v_prev] [dt_ms]\n"
            "Optional named args:\n"
            "  --Ts <°C>      system temperature (default 155.0)\n"
            "  --Th <°C>      hot-leg temperature (default 35.0)\n"
            "  --Tc <°C>      cold-leg temperature (default 25.0)\n"
            "  --v_prev <rpm> last fan speed (default 0.0)\n"
            "  --dt_ms <ms>   fixed timestep (default auto)\n"
            "  --mdot <kg/s>  flow rate (default 0.18)\n",
            argv[0]);
        return 1;
    }

    const char* ifname = argv[1];

    // ---- Default values ----
    double Ts_init = 155.0;
    double Th_init = 35.0;
    double Tc_init = 25.0;
    double vprev_init = 0.0;
    double dt_fixed_s = -1.0;
    double mdot_init = 0.18;

    // ---- Positional backward compatibility ----

    if (argc > 3) Th_init     = parse_or(argv[3], Th_init);
    if (argc > 4) vprev_init  = parse_or(argv[4], vprev_init);
    if (argc > 5) {
        double ms = parse_or(argv[5], -1.0);
        if (ms >= 1.0 && ms <= 255.0) dt_fixed_s = ms * 1e-3;
    }

    // ---- Parse named args (any order) ----
    for (int i = 2; i < argc - 1; i++) {
        if      (strcmp(argv[i], "--Ts")     == 0) Ts_init     = parse_or(argv[i+1], Ts_init);
        else if (strcmp(argv[i], "--Th")     == 0) Th_init     = parse_or(argv[i+1], Th_init);
        else if (strcmp(argv[i], "--Tc")     == 0) Tc_init     = parse_or(argv[i+1], Tc_init);
        else if (strcmp(argv[i], "--v_prev") == 0) vprev_init  = parse_or(argv[i+1], vprev_init);
        else if (strcmp(argv[i], "--dt_ms")  == 0) {
            double ms = parse_or(argv[i+1], -1.0);
            if (ms >= 1.0 && ms <= 255.0) dt_fixed_s = ms * 1e-3;
        }
        else if (strcmp(argv[i], "--mdot")   == 0) mdot_init   = parse_or(argv[i+1], mdot_init);
    }

    // ---- Socket setup (unchanged) ----
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) die("socket");

    struct can_filter flt;
    flt.can_id = 0x201; flt.can_mask = CAN_SFF_MASK;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, sizeof(flt)) < 0) die("setsockopt");
    bind_socket(s, ifname);

    printf("[C/Plant] RX 0x201 (omega_cmd,v_cmd), TX 0x202 (Ts,Th,Tc,v_prev,dt)\n");

    // ---- Initial plant state ----
    Plant st = {
        .Ts    = Ts_init,
        .Th    = Th_init,
        .Tc    = Tc_init,
        .mdot  = mdot_init,
        .v_prev= vprev_init
    };

    uint64_t next_print = now_ms() + 500;


    for(;;){
        // poll for command frame with a small timeout (e.g., 50 ms)
        struct pollfd pfd = { .fd = s, .events = POLLIN };
        int pr = poll(&pfd, 1, 50);
        if (pr < 0) die("poll");

        // compute dt since last loop (never 0)
        double dt = dt_fixed_s;
        if (dt < 0.0005) dt = 0.0005;     // 0.5 ms minimum
        if (dt > 0.255)  dt = 0.255;      // cap to 255 ms (fits in uint8)

        double omega_cmd = 0.0, v_cmd = st.v_prev; // default to last v if nothing received

        if (pr > 0 && (pfd.revents & POLLIN)) {
            struct can_frame f;
            if (recv(s, &f, sizeof(f), 0) < 0) die("recv");
        
            printf("[C] RX 0x%03X [%d]:", f.can_id & CAN_SFF_MASK, f.len);
            for (int i = 0; i < f.len && i < 8; i++) printf(" %02X", f.data[i]);
            printf("\n");
        
            if ((f.can_id & CAN_SFF_MASK) == 0x201 && f.len >= 4) {
                uint16_t om = f.data[0] | (f.data[1] << 8);
                uint16_t vc = f.data[2] | (f.data[3] << 8);
                omega_cmd = sat((double)om, 0, omega_max);
                v_cmd     = sat((double)vc, 0, v_max);
                printf("→ omega=%.0f rpm, v=%.0f rpm\n", omega_cmd, v_cmd);
            }
        }

        // integrate plant one step with commands
        plant_step(&st, omega_cmd, v_cmd, dt);

        // Build feedback frame (one 8-byte message)
        struct can_frame tx = {0};
        tx.can_id = 0x202; tx.len = 8;

        int16_t Ts_q = pack_temp_q10(st.Ts);
        int16_t Th_q = pack_temp_q10(st.Th);
        int16_t Tc_q = pack_temp_q10(st.Tc);
        uint8_t vprev_q = pack_v_prev_q10(st.v_prev);
        uint8_t dt_q = pack_dt_ms(dt);

        le_from_u16(&tx.data[0], &tx.data[1], (uint16_t)Ts_q);
        le_from_u16(&tx.data[2], &tx.data[3], (uint16_t)Th_q);
        le_from_u16(&tx.data[4], &tx.data[5], (uint16_t)Tc_q);
        tx.data[6] = vprev_q;
        tx.data[7] = dt_q;

        if (send(s, &tx, sizeof(tx), 0) < 0) die("send");

        // light console print
        uint64_t nowm = now_ms();
        if ((int64_t)(nowm - next_print) >= 0){
            printf("[C/Plant] Ts=%.1f Th=%.1f Tc=%.1f mdot=%.3f  v=%.0f rpm  dt=%ums  | omega_cmd=%.0f v_cmd=%.0f\n",
                   st.Ts, st.Th, st.Tc, st.mdot, st.v_prev, (unsigned)dt_q,
                   omega_cmd, v_cmd);
            next_print = nowm + 500;
        }
    }
    return 0;
}
#endif