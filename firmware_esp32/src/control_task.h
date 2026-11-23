// =============================================================================
//  control_task.h — Task điều khiển thời gian thực (Core 1, CONTROL_LOOP_HZ):
//  đọc encoder -> vận tốc bánh -> PID -> PWM, tích phân odometry, failsafe.
// =============================================================================
#pragma once

namespace control {

// Gọi ĐẦU TIÊN trong setup(): đưa driver motor về trạng thái an toàn và gắn
// ngắt encoder (ISR chạy trên core gọi hàm này = Core 1 của loopTask).
void init();

// Tạo task điều khiển, ghim vào CONTROL_TASK_CORE.
void start();

// Độ sâu stack còn trống nhỏ nhất của task (byte) — để chẩn đoán.
unsigned stackHighWaterMark();

}  // namespace control
