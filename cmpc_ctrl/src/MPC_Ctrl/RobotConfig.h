#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

namespace RobotConfig {

    // =========================================================================
    // CHỌN CẤU HÌNH ROBOT TẠI ĐÂY (BẬT / TẮT 1 DÒNG DUY NHẤT)
    // =========================================================================
    // #define LITE3_WITH_PAYLOAD_2KG  // Bỏ comment nếu robot gắn thêm 2kg payload

#ifndef LITE3_WITH_PAYLOAD_2KG
    // -------------------------------------------------------------------------
    // 1. Cấu hình tiêu chuẩn Lite3 (Không tải - Nominal)
    // -------------------------------------------------------------------------
    constexpr float MASS = 11.94f;            // Tổng khối lượng robot [kg] (TORSO + 4 chân)
    constexpr float IXX  = 0.171f;            // Quán tính trục Roll [kg.m^2]
    constexpr float IYY  = 0.301f;            // Quán tính trục Pitch [kg.m^2]
    constexpr float IZZ  = 0.372f;            // Quán tính trục Yaw [kg.m^2]
    constexpr bool isComOffset = false;
#else
    // -------------------------------------------------------------------------
    // 2. Cấu hình Lite3 CÓ TẢI 2 KG (gắn lệch phía trước 10 cm)
    // -------------------------------------------------------------------------
    constexpr float MASS = 13.94f;            // Tổng khối lượng [kg] (11.94 kg + 2 kg)
    constexpr float IXX  = 0.180f;            // Quán tính trục Roll [kg.m^2]
    constexpr float IYY  = 0.330f;            // Quán tính trục Pitch [kg.m^2]
    constexpr float IZZ  = 0.393f;            // Quán tính trục Yaw [kg.m^2]
    constexpr bool isComOffset = true;
#endif

    // -------------------------------------------------------------------------
    // 3. Tham số bài toán tối ưu Convex MPC
    // -------------------------------------------------------------------------
    constexpr float BODY_HEIGHT    = 0.29f;   // Chiều cao thân robot danh định [m]
    constexpr float MAX_FORCE      = 120.0f;  // Lực pháp tuyến (Fz) tối đa mỗi chân [N]
    constexpr float FRICTION_MU    = 0.4f;    // Hệ số ma sát mặt đất (mu)
    constexpr float MPC_ALPHA      = 4e-7f;   // Trọng số chính quy hóa lực (R = alpha * I)

} // namespace RobotConfig

#endif // _ROBOT_CONFIG_H_
