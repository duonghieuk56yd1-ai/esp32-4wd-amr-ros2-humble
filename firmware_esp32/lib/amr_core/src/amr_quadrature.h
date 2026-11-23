// =============================================================================
//  amr_quadrature.h — Bảng giải mã encoder quadrature x4 (logic thuần, test được
//  trên PC). Trạng thái 2 bit = (A << 1) | B.
//
//  Chiều thuận: 00 -> 01 -> 11 -> 10 -> 00  (mỗi bước +1, 4 bước/xung)
//  Chiều nghịch: 00 -> 10 -> 11 -> 01 -> 00 (mỗi bước -1)
//  Hai bit cùng đổi một lúc = mất cạnh (kInvalid) -> không đếm, chỉ thống kê lỗi.
//
//  16 mục x 2 bit được nén vào MỘT hằng số 32 bit (kLut) để ISR trong IRAM tra
//  bảng mà không phải đọc mảng nằm trong flash.
// =============================================================================
#pragma once

#include <stdint.h>

namespace amr {
namespace quad {

enum : uint8_t {
  kNone = 0,     // không đổi trạng thái
  kForward = 1,  // +1
  kReverse = 2,  // -1
  kInvalid = 3,  // nhảy 2 bước (mất cạnh)
};

namespace detail {
// Chỉ số = (trạng_thái_cũ << 2) | trạng_thái_mới
constexpr uint8_t kTable[16] = {
    // mới:  00        01        10        11
    kNone,    kForward, kReverse, kInvalid,  // cũ 00
    kReverse, kNone,    kInvalid, kForward,  // cũ 01
    kForward, kInvalid, kNone,    kReverse,  // cũ 10
    kInvalid, kReverse, kForward, kNone,     // cũ 11
};

constexpr uint32_t pack(int i = 0) {
  return i >= 16 ? 0u : ((static_cast<uint32_t>(kTable[i]) << (2 * i)) | pack(i + 1));
}
}  // namespace detail

// Bảng nén: mã chuyển trạng thái của chỉ số i nằm ở bit [2i+1 : 2i].
constexpr uint32_t kLut = detail::pack();
static_assert(kLut == 0x1B8D72E4u, "quadrature LUT bị sai");

// Mã chuyển trạng thái (kNone/kForward/kReverse/kInvalid) cho prev -> curr.
constexpr uint8_t transition(uint8_t prev_ab, uint8_t curr_ab) {
  return static_cast<uint8_t>(
      (kLut >> ((((prev_ab & 3u) << 2) | (curr_ab & 3u)) << 1)) & 3u);
}

// Giá trị cộng vào bộ đếm ứng với một mã chuyển trạng thái.
constexpr int8_t delta(uint8_t code) {
  return code == kForward ? 1 : (code == kReverse ? -1 : 0);
}

// Số tick đã đi giữa hai lần đọc bộ đếm 32 bit, đúng cả khi bộ đếm tràn
// (INT32_MAX -> INT32_MIN) nhờ phép trừ không dấu.
constexpr int32_t tickDelta(int32_t now, int32_t prev) {
  return static_cast<int32_t>(static_cast<uint32_t>(now) - static_cast<uint32_t>(prev));
}

// Đổi số tick (đã giải mã x4) sang góc quay bánh xe (rad).
constexpr double ticksToRadians(int64_t ticks, int32_t ticks_per_rev) {
  return static_cast<double>(ticks) * (2.0 * 3.14159265358979323846) /
         static_cast<double>(ticks_per_rev);
}

}  // namespace quad
}  // namespace amr
