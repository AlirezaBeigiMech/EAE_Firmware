// plant_user_test.cc
#include <gtest/gtest.h>
extern "C" {
  #include "plant_user_api.h"
}

TEST(ParseOr, ValidNumbersAndFallbacks) {
  EXPECT_DOUBLE_EQ(parse_or("3.5", 1.0), 3.5);
  EXPECT_DOUBLE_EQ(parse_or("",    2.0), 2.0);
  EXPECT_DOUBLE_EQ(parse_or(nullptr, 7.0), 7.0);
  EXPECT_DOUBLE_EQ(parse_or("abc", 5.5), 5.5);
}

TEST(Sat, ClampsProperly) {
  EXPECT_DOUBLE_EQ(sat(5.0, 0.0, 10.0), 5.0);
  EXPECT_DOUBLE_EQ(sat(-1.0, 0.0, 10.0), 0.0);
  EXPECT_DOUBLE_EQ(sat(15.0, 0.0, 10.0), 10.0);
}

TEST(SoftAbs, BehavesNearZero) {
  const double eps = 1e-6;
  EXPECT_NEAR(softabs(0.0, eps), eps, eps*1e-12);
  EXPECT_NEAR(softabs(3.0, eps), 3.0, 1e-12);
}

TEST(SafeSq, CapsBeforeSquaring) {
  EXPECT_DOUBLE_EQ(safe_sq(3.0, 10.0), 9.0);
  EXPECT_DOUBLE_EQ(safe_sq(20.0, 10.0), 100.0);
  EXPECT_DOUBLE_EQ(safe_sq(-25.0, 10.0), 100.0);
}

TEST(WaterViscosity, MonotoneDecreasingWithTemperature) {
  double mu20 = mu_water(20.0);
  double mu60 = mu_water(60.0);
  double mu90 = mu_water(90.0);
  EXPECT_GT(mu20, mu60);
  EXPECT_GT(mu60, mu90);
  EXPECT_GT(mu20, mu90);
}

TEST(UAfunc, IncreasesWithFanSpeedAndIsClamped) {
  double ua0 = UA_func(0.0,  60.0);
  double ua1 = UA_func(100.0,60.0);
  double ua2 = UA_func(600.0,60.0);
  EXPECT_GE(ua0, 1.0);
  EXPECT_LT(ua0, ua1);
  EXPECT_LE(ua1, ua2);
  // should be clamped to [1, 5e3]
  EXPECT_GE(UA_func(-50.0,60.0), 1.0);
  EXPECT_LE(UA_func(1e6, 60.0), 5000.0);
}

TEST(Psys, NonNegativeAndGrowsWithTs) {
  EXPECT_GE(Psys(0.0, 25.0), 0.0);
  EXPECT_LT(Psys(0.0, 25.0), Psys(0.0, 100.0));
  EXPECT_LE(Psys(0.0, 2000.0), 2e5); // capped
}


TEST(Packing, VprevAndDtQuantization) {
  // v_prev: q = round(v_rpm/10)
  EXPECT_EQ(pack_v_prev_q10(0.0),   0);
  EXPECT_EQ(pack_v_prev_q10(55.0),  6);   // 55/10 ≈ 5.5 → 6
  EXPECT_EQ(pack_v_prev_q10(2550.0),255); // clamp
  EXPECT_EQ(pack_v_prev_q10(9000.0),255); // clamp
  // dt_ms: clamp [1,255]
  EXPECT_EQ(pack_dt_ms(0.0004), 1);
  EXPECT_EQ(pack_dt_ms(0.0100), 10);
  EXPECT_EQ(pack_dt_ms(1.0000), 255);
}

TEST(PlantStep, StableNoCommandSmallDt) {
  Plant s{.Ts=60.0, .Th=40.0, .Tc=30.0, .mdot=0.18, .v_prev=0.0};
  const double omega_cmd = 0.0;   // pump off
  const double v_cmd     = 0.0;   // fan off
  const double dt        = 0.01;  // 10 ms
  // Run a few steps. Temperatures should not explode; Ts should tend to cool down slowly.
  for (int i = 0; i < 100; ++i) {
    plant_step(&s, omega_cmd, v_cmd, dt);
    EXPECT_GE(s.Ts, -400.0);
    EXPECT_LE(s.Ts, 1500.0);
    EXPECT_GE(s.Th, -400.0);
    EXPECT_GE(s.Tc, -400.0);
    EXPECT_GE(s.mdot, 0.0);
  }
  // Qualitative check: with no pump/fan, Ts generally shouldn’t increase without bound
  // (it may drift based on Psys vs losses, but bounded by clamps).
  EXPECT_LE(s.Ts, 200.0);
}

TEST(PlantStep, FanIncreasesUAAndCoolsColdLegFaster) {
  // identical initial states; one with v=0, the other with v=1000 rpm
  Plant a{.Ts=80.0, .Th=60.0, .Tc=50.0, .mdot=0.18, .v_prev=0.0};
  Plant b = a;
  const double omega_cmd = 2000.0;     // some pump speed
  const double dt = 0.02;              // 20 ms
  for (int i = 0; i < 250; ++i) {      // 5 seconds
    plant_step(&a, omega_cmd, 0.0,    dt);
    plant_step(&b, omega_cmd, 1000.0, dt);
  }
  // With fan, the radiator UA is higher → Tc should be lower (better cooling)
  EXPECT_LT(b.Tc, a.Tc);
}
