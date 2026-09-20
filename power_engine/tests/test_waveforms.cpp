#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>

#include "power_engine/waveforms.h"

using power_engine::waveforms::Waveform;

TEST(Waveforms, ConstantAndSine) {
  auto c = Waveform::constant(2.5);
  EXPECT_DOUBLE_EQ(c.value(0.0), 2.5);
  EXPECT_DOUBLE_EQ(c.value(1e3), 2.5);
  auto s = Waveform::sin(1000.0, 0.1, 1.0, 0.0);
  EXPECT_NEAR(s.value(0.0), 1.0, 1e-12);
  EXPECT_NEAR(s.value(0.00025), 1.1, 1e-12);  // quarter period peak
  auto q = Waveform::sin(1000.0, 0.1, 0.0, 3.14159265358979 / 2.0);
  EXPECT_NEAR(q.value(0.0), 0.1, 1e-9);
  EXPECT_THROW(Waveform::sin(0.0, 1.0), std::runtime_error);
  EXPECT_THROW(Waveform::constant(std::numeric_limits<double>::infinity()),
               std::runtime_error);
}

TEST(Waveforms, PulseTrain) {
  // 5V pulses, 10us wide every 50us from t=1ms, infinite.
  auto p = Waveform::pulse(1e-3, 10e-6, 50e-6, 5.0, 0.0);
  EXPECT_DOUBLE_EQ(p.value(0.0), 0.0);
  EXPECT_DOUBLE_EQ(p.value(1e-3), 5.0);
  EXPECT_DOUBLE_EQ(p.value(1e-3 + 9.9e-6), 5.0);
  EXPECT_DOUBLE_EQ(p.value(1e-3 + 10.1e-6), 0.0);
  EXPECT_DOUBLE_EQ(p.value(1e-3 + 50e-6 + 1e-9), 5.0);  // 2nd pulse, off-boundary
  // Finite: exactly 2 pulses, then dc forever.
  auto d = Waveform::pulse(0.0, 10e-6, 50e-6, 5.0, 1.0, 2);
  EXPECT_DOUBLE_EQ(d.value(5e-6), 6.0);
  EXPECT_DOUBLE_EQ(d.value(55e-6), 6.0);
  EXPECT_DOUBLE_EQ(d.value(105e-6), 1.0);
  EXPECT_DOUBLE_EQ(d.value(1.0), 1.0);
  EXPECT_THROW(Waveform::pulse(0.0, 50e-6, 50e-6, 1.0), std::runtime_error);
  EXPECT_THROW(Waveform::pulse(0.0, 10e-6, 50e-6, 1.0, 0.0, 0), std::runtime_error);
}

TEST(Waveforms, PwlSegments) {
  auto w = Waveform::pwl({0.0, 1e-3, 3e-3}, {0.0, 10.0, 10.0});
  EXPECT_DOUBLE_EQ(w.value(-1.0), 0.0);  // hold first
  EXPECT_DOUBLE_EQ(w.value(0.0), 0.0);
  EXPECT_DOUBLE_EQ(w.value(0.5e-3), 5.0);  // ramp midpoint
  EXPECT_DOUBLE_EQ(w.value(2e-3), 10.0);
  EXPECT_DOUBLE_EQ(w.value(1.0), 10.0);  // hold last
  EXPECT_THROW(Waveform::pwl({}, {}), std::runtime_error);
  EXPECT_THROW(Waveform::pwl({0.0}, {1.0, 2.0}), std::runtime_error);
  EXPECT_THROW(Waveform::pwl({1.0, 0.0}, {1.0, 2.0}), std::runtime_error);
}
