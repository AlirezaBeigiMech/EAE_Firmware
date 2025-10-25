#include <gtest/gtest.h>
#include <cmath>
extern "C" {
  #include "ctrl_set_api.h"
}

static uint16_t U16(const uint8_t lo, const uint8_t hi){ return (uint16_t)(lo | (hi<<8)); }

TEST(CtrlSetQuant, Q01RoundsAndClamps) {
  EXPECT_EQ(to_q01( 30.0f), 300);       // 30.0°C -> 300
  EXPECT_EQ(to_q01(-4000.0f), -32768);  // clamp low
  EXPECT_EQ(to_q01( 4000.0f),  32767);  // clamp high
  EXPECT_EQ(to_q01( 30.04f),  300);     // 300.4 -> 300 (round)
  EXPECT_EQ(to_q01( 30.05f),  301);     // 300.5 -> 301 (round half up)
}

TEST(CtrlSetQuant, Q88AndQ44Basics) {
  EXPECT_EQ(to_q88(1.0f), 256u);
  EXPECT_EQ(to_q88(0.0f), 0u);
  EXPECT_EQ(to_q88(-2.0f), 0u);         // clamp low
  EXPECT_EQ(to_q88(1000.0f), 65535u);   // clamp high

  EXPECT_EQ(to_q44(1.0f), 16u);
  EXPECT_EQ(to_q44(0.0f), 0u);
  EXPECT_EQ(to_q44(-3.0f), 0u);         // clamp low
  EXPECT_EQ(to_q44(100.0f), 255u);      // clamp high
}

TEST(CtrlSetFrames, BuildSetpointOnly) {
  can_frame sp{}, p1{}, p2{};
  CtrlParams P{};
  P.Ts_sp_C = 30.0f;
  // Gains
  P.KpT = 0.0f;  P.KiT = 0.0f;  P.KdT = 0.0f;   P.kawT = 0.0f;
  P.Kpm = 0.0f;  P.Kim = 0.0f;  P.kawm = 0.0f;  P.kvw  = 0.0f;  P.kwv = 0.0f;
  P.send_params = false;

  build_ctrl_frames(&P, &sp, &p1, &p2);
  EXPECT_EQ(sp.can_id, 0x301u);
  EXPECT_EQ(sp.len, 8);
  EXPECT_EQ(U16(sp.data[0], sp.data[1]), (uint16_t)300);  // 30.0°C -> q0.1 = 300

  // params disabled -> untouched
  EXPECT_EQ(p1.can_id, 0u);
  EXPECT_EQ(p2.can_id, 0u);
}

TEST(CtrlSetFrames, BuildParamsAll) {
  can_frame sp{}, p1{}, p2{};

  CtrlParams P{};
  P.Ts_sp_C = 30.0f;
  // Gains
  P.KpT = 120.0f;  P.KiT = 0.15f;  P.KdT = 5.0f;   P.kawT = 4.0f;
  P.Kpm = 150.0f;  P.Kim = 0.02f;  P.kawm = 8.0f;  P.kvw  = -0.1f;  P.kwv = -0.03f;
  P.send_params = true;

  build_ctrl_frames(&P, &sp, &p1, &p2);

  // 0x301
  EXPECT_EQ(sp.can_id, 0x301u);
  EXPECT_EQ(U16(sp.data[0], sp.data[1]), (uint16_t)300);

  // 0x300: KpT/KiT/KdT q8.8, kawT q4.4
  EXPECT_EQ(p1.can_id, 0x300u);
  EXPECT_EQ(p1.len, 8);
  EXPECT_EQ(U16(p1.data[0], p1.data[1]), (uint16_t)lround(120.0f*256.0f));
  EXPECT_EQ(U16(p1.data[2], p1.data[3]), (uint16_t)lround(  0.15f*256.0f));
  EXPECT_EQ(U16(p1.data[4], p1.data[5]), (uint16_t)lround(  5.00f*256.0f));
  EXPECT_EQ(p1.data[6], (uint8_t)lround(4.0f*16.0f));

  // 0x302: Kpm/Kim q8.8, kawm/kvw/kwv q4.4 (negatives clamp to 0 per to_q44)
  EXPECT_EQ(p2.can_id, 0x302u);
  EXPECT_EQ(p2.len, 8);
  EXPECT_EQ(U16(p2.data[0], p2.data[1]), (uint16_t)lround(150.0f*256.0f));
  EXPECT_EQ(U16(p2.data[2], p2.data[3]), (uint16_t)lround(  0.02f*256.0f));
  EXPECT_EQ(p2.data[4], (uint8_t)lround(8.0f*16.0f));
  EXPECT_EQ(p2.data[5], 0u); // -0.1 → clamp to 0
  EXPECT_EQ(p2.data[6], 0u); // -0.03 → clamp to 0
}
