// Unit test: giải mã encoder quadrature x4 và đổi tick -> góc bánh (rad).
// Chạy trên PC: pio test -e native -f test_quadrature
#include <unity.h>

#include <climits>
#include <cmath>
#include <cstdint>
#include <vector>

#include "amr_quadrature.h"
#include "robot_config.h"

void setUp() {}
void tearDown() {}

namespace {

// Mô phỏng ISR: trả về số đếm sau khi đi qua chuỗi trạng thái AB.
int32_t runQuadrature(const std::vector<uint8_t>& states, uint32_t* invalid = nullptr) {
  int32_t count = 0;
  uint8_t prev = states.front();
  for (size_t i = 1; i < states.size(); ++i) {
    const uint8_t code = amr::quad::transition(prev, states[i]);
    if (code == amr::quad::kInvalid && invalid) ++(*invalid);
    count += amr::quad::delta(code);
    prev = states[i];
  }
  return count;
}

// Chuỗi trạng thái AB của n chu kỳ xung đầy đủ (4 cạnh / chu kỳ).
std::vector<uint8_t> pulses(int n, bool forward) {
  static const uint8_t kFwd[4] = {0b01, 0b11, 0b10, 0b00};
  static const uint8_t kRev[4] = {0b10, 0b11, 0b01, 0b00};
  std::vector<uint8_t> s{0b00};
  for (int i = 0; i < n; ++i) {
    for (int k = 0; k < 4; ++k) s.push_back(forward ? kFwd[k] : kRev[k]);
  }
  return s;
}

}  // namespace

void test_quadrature_forward_and_reverse() {
  // 10 xung đủ chu kỳ = 40 tick (giải mã x4)
  TEST_ASSERT_EQUAL_INT32(40, runQuadrature(pulses(10, true)));
  TEST_ASSERT_EQUAL_INT32(-40, runQuadrature(pulses(10, false)));
}

void test_quadrature_one_wheel_revolution() {
  // Một vòng bánh JGA25-370 1:34 = ENCODER_PPR xung = ENCODER_TICKS_PER_REV tick
  TEST_ASSERT_EQUAL_INT32(ENCODER_TICKS_PER_REV, runQuadrature(pulses(ENCODER_PPR, true)));
  TEST_ASSERT_EQUAL_INT32(1496, ENCODER_TICKS_PER_REV);
}

void test_quadrature_bounce_cancels_and_invalid_flagged() {
  // Nảy cạnh trên kênh B: 00 -> 01 -> 00 -> 01 -> 11 : thực chất tiến 2 bước
  TEST_ASSERT_EQUAL_INT32(2, runQuadrature({0b00, 0b01, 0b00, 0b01, 0b11}));

  uint32_t invalid = 0;
  // 00 -> 11 và 01 -> 10 là nhảy 2 bước: không đếm, chỉ báo lỗi
  TEST_ASSERT_EQUAL_INT32(0, runQuadrature({0b00, 0b11, 0b11}, &invalid));
  TEST_ASSERT_EQUAL_UINT32(1, invalid);
  TEST_ASSERT_EQUAL_UINT8(amr::quad::kInvalid, amr::quad::transition(0b01, 0b10));
  TEST_ASSERT_EQUAL_UINT8(amr::quad::kNone, amr::quad::transition(0b10, 0b10));
}

// Bảng tra phải phản đối xứng: a->b rồi b->a luôn về 0, đứng yên không đếm,
// đổi cả 2 bit cùng lúc là lỗi (mất cạnh).
void test_quadrature_table_is_antisymmetric() {
  for (uint8_t a = 0; a < 4; ++a) {
    TEST_ASSERT_EQUAL_UINT8(amr::quad::kNone, amr::quad::transition(a, a));
    for (uint8_t b = 0; b < 4; ++b) {
      const uint8_t ab = amr::quad::transition(a, b);
      const uint8_t ba = amr::quad::transition(b, a);
      TEST_ASSERT_EQUAL_INT(-amr::quad::delta(ab), amr::quad::delta(ba));
      const bool both_bits_changed = ((a ^ b) & 3u) == 3u;
      TEST_ASSERT_EQUAL_INT(both_bits_changed ? 1 : 0, ab == amr::quad::kInvalid ? 1 : 0);
    }
  }
}

// Bộ đếm 32 bit tràn (INT32_MAX -> INT32_MIN) không được làm nhảy odometry.
void test_tick_delta_handles_counter_wraparound() {
  TEST_ASSERT_EQUAL_INT32(60, amr::quad::tickDelta(100, 40));
  TEST_ASSERT_EQUAL_INT32(-60, amr::quad::tickDelta(40, 100));
  TEST_ASSERT_EQUAL_INT32(10, amr::quad::tickDelta(INT32_MIN + 4, INT32_MAX - 5));
  TEST_ASSERT_EQUAL_INT32(-10, amr::quad::tickDelta(INT32_MAX - 5, INT32_MIN + 4));

  // Mô phỏng ISR đếm qua điểm tràn trong khi vòng điều khiển đọc định kỳ
  uint32_t raw = static_cast<uint32_t>(INT32_MAX) - 1000u;
  int32_t prev = static_cast<int32_t>(raw);
  int64_t total = 0;
  for (int i = 0; i < 100; ++i) {
    raw += 37u;  // 37 tick mỗi chu kỳ điều khiển
    const int32_t now = static_cast<int32_t>(raw);
    total += amr::quad::tickDelta(now, prev);
    prev = now;
  }
  TEST_ASSERT_TRUE(total == 3700);
}

void test_ticks_to_radians() {
  const double two_pi = 2.0 * M_PI;
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, two_pi, amr::quad::ticksToRadians(1496, 1496));
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, -M_PI, amr::quad::ticksToRadians(-748, 1496));
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, amr::quad::ticksToRadians(0, 1496));
  // Tổng tick 64 bit: 10^6 vòng bánh vẫn chính xác
  const int64_t many = 1000000LL * ENCODER_TICKS_PER_REV;
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1e6 * two_pi,
                            amr::quad::ticksToRadians(many, ENCODER_TICKS_PER_REV));
  // 1 tick ~ 0.136 mm quãng đường lăn với bánh r = 3.25 cm
  const double mm_per_tick =
      1000.0 * WHEEL_RADIUS_M * amr::quad::ticksToRadians(1, ENCODER_TICKS_PER_REV);
  TEST_ASSERT_DOUBLE_WITHIN(1e-3, 0.1365, mm_per_tick);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_quadrature_forward_and_reverse);
  RUN_TEST(test_quadrature_one_wheel_revolution);
  RUN_TEST(test_quadrature_bounce_cancels_and_invalid_flagged);
  RUN_TEST(test_quadrature_table_is_antisymmetric);
  RUN_TEST(test_tick_delta_handles_counter_wraparound);
  RUN_TEST(test_ticks_to_radians);
  return UNITY_END();
}
