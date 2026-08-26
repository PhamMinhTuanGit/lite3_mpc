#include <iostream>
#include <cmath>
#include <array>
#include <Eigen/Core>

#include "WbcType.h"
#include "WbicController.hpp"

int main()
{
    std::cout << "[TEST] wbic_mapping_smoke starting..." << std::endl;

    // 1. Verify Sentinel Mapping between RI and CMPC/WBIC
    // RI order:   [FL (0), FR (1), HL (2), HR (3)]
    // CMPC order: [FR (0), FL (1), HR (2), HL (3)]
    constexpr int kCmpcToRiLeg[4] = {1, 0, 3, 2};
    constexpr int kRiToCmpcLeg[4] = {1, 0, 3, 2};

    // Forward and inverse consistency
    for (int cmpc_leg = 0; cmpc_leg < 4; ++cmpc_leg) {
        const int ri_leg = kCmpcToRiLeg[cmpc_leg];
        const int recovered_cmpc_leg = kRiToCmpcLeg[ri_leg];
        if (cmpc_leg != recovered_cmpc_leg) {
            std::cerr << "Inconsistent leg remapping for leg " << cmpc_leg << std::endl;
            return 1;
        }
    }

    // 2. Sentinel value propagation
    // Create distinct sensor values in RI order
    double ri_pos[12] = {};
    double ri_vel[12] = {};
    for (int ri_leg = 0; ri_leg < 4; ++ri_leg) {
        for (int j = 0; j < 3; ++j) {
            ri_pos[ri_leg * 3 + j] = (ri_leg + 1) * 10.0 + j;
            ri_vel[ri_leg * 3 + j] = (ri_leg + 1) * 1.0 + 0.1 * j;
        }
    }

    // Convert RI -> CMPC/WBIC format
    wbic::WbicInput input;
    for (int cmpc_leg = 0; cmpc_leg < 4; ++cmpc_leg) {
        const int ri_leg = kCmpcToRiLeg[cmpc_leg];
        for (int j = 0; j < 3; ++j) {
            input.q_joint_raw[cmpc_leg * 3 + j] = ri_pos[ri_leg * 3 + j];
            input.qd_joint_raw[cmpc_leg * 3 + j] = ri_vel[ri_leg * 3 + j];
        }
    }

    // Verify mapped values match expected physical joints
    // FR is cmpc_leg 0, which corresponds to ri_leg 1 (FR) -> values 20, 21, 22
    if (std::abs(input.q_joint_raw[0 * 3 + 0] - 20.0) > 1e-9 ||
        std::abs(input.q_joint_raw[0 * 3 + 1] - 21.0) > 1e-9 ||
        std::abs(input.q_joint_raw[0 * 3 + 2] - 22.0) > 1e-9) {
        std::cerr << "FR mapping mismatch!" << std::endl;
        return 2;
    }

    // FL is cmpc_leg 1, which corresponds to ri_leg 0 (FL) -> values 10, 11, 12
    if (std::abs(input.q_joint_raw[1 * 3 + 0] - 10.0) > 1e-9 ||
        std::abs(input.q_joint_raw[1 * 3 + 1] - 11.0) > 1e-9 ||
        std::abs(input.q_joint_raw[1 * 3 + 2] - 12.0) > 1e-9) {
        std::cerr << "FL mapping mismatch!" << std::endl;
        return 3;
    }

    // 3. Test WBIC Controller Output to Hybrid Command remap
    wbic::WbicConfig cfg;
    wbic::WbicController wbic_ctrl(cfg);

    wbic::WbicOutput output;
    output.reset();
    for (int cmpc_leg = 0; cmpc_leg < 4; ++cmpc_leg) {
        for (int j = 0; j < 3; ++j) {
            output.joint_kp[cmpc_leg * 3 + j] = 3.0;
            output.q_des[cmpc_leg * 3 + j] = (cmpc_leg + 1) * 100.0 + j;
            output.joint_kd[cmpc_leg * 3 + j] = 0.2;
            output.dq_des[cmpc_leg * 3 + j] = (cmpc_leg + 1) * 5.0 + j;
            output.tau_ff[cmpc_leg * 3 + j] = (cmpc_leg + 1) * 2.0 + j; // Positive torque
        }
    }

    wbic::JointHybridCommand hybrid_cmpc;
    wbic::WbicController::PopulateHybridCommand(output, &hybrid_cmpc);

    // Apply remap from CMPC order [FR, FL, HR, HL] to RI order [FL, FR, HL, HR]
    Eigen::Matrix<float, 12, 5> joint_cmd;
    joint_cmd.setZero();
    for (int cmpc_leg = 0; cmpc_leg < 4; ++cmpc_leg) {
        const int ri_leg = kCmpcToRiLeg[cmpc_leg];
        for (int j = 0; j < 3; ++j) {
            joint_cmd(ri_leg * 3 + j, 0) = static_cast<float>(hybrid_cmpc.kp[cmpc_leg * 3 + j]);
            joint_cmd(ri_leg * 3 + j, 1) = static_cast<float>(hybrid_cmpc.q_des[cmpc_leg * 3 + j]);
            joint_cmd(ri_leg * 3 + j, 2) = static_cast<float>(hybrid_cmpc.kd[cmpc_leg * 3 + j]);
            joint_cmd(ri_leg * 3 + j, 3) = static_cast<float>(hybrid_cmpc.qd_des[cmpc_leg * 3 + j]);
            // ★ WBIC torque is NOT inverted on HipX
            joint_cmd(ri_leg * 3 + j, 4) = static_cast<float>(hybrid_cmpc.tau_ff[cmpc_leg * 3 + j]);
        }
    }

    // Verify RI FL joints (ri_leg 0) received cmpc_leg 1 (FL) values (100-series)
    if (std::abs(joint_cmd(0 * 3 + 0, 1) - 200.0f) > 1e-4 ||
        std::abs(joint_cmd(0 * 3 + 1, 1) - 201.0f) > 1e-4 ||
        std::abs(joint_cmd(0 * 3 + 2, 1) - 202.0f) > 1e-4) {
        std::cerr << "FL joint_cmd remapping error: " << joint_cmd.row(0) << std::endl;
        return 4;
    }

    // Verify RI FR joints (ri_leg 1) received cmpc_leg 0 (FR) values (100-series)
    if (std::abs(joint_cmd(1 * 3 + 0, 1) - 100.0f) > 1e-4 ||
        std::abs(joint_cmd(1 * 3 + 1, 1) - 101.0f) > 1e-4 ||
        std::abs(joint_cmd(1 * 3 + 2, 1) - 102.0f) > 1e-4) {
        std::cerr << "FR joint_cmd remapping error: " << joint_cmd.row(3) << std::endl;
        return 5;
    }

    // Verify HipX torque sign is preserved
    if (joint_cmd(0 * 3 + 0, 4) <= 0.0f || joint_cmd(1 * 3 + 0, 4) <= 0.0f) {
        std::cerr << "HipX torque sign flipped incorrectly" << std::endl;
        return 6;
    }

    std::cout << "[TEST] wbic_mapping_smoke PASSED!" << std::endl;
    return 0;
}
