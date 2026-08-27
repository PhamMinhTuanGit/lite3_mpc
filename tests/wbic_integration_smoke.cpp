#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

#include "GaitCtrller.h"
#include "WbcType.h"

int main()
{
    std::cout << "[TEST] wbic_integration_smoke starting..." << std::endl;

    double pidParam[4] = {100.0, 1.0, 0.0, 0.05};
    GaitCtrller controller(500.0, pidParam, true);

    controller.SetGaitType(0); // trot
    controller.SetRobotMode(0); // follow velocity

    double target_vel[3] = {0.0, 0.0, 0.0}; // Steady standing
    controller.SetRobotVel(target_vel);

    // Initial state
    double imuData[10] = {
        0.0, 0.0, 9.81, // acc
        0.0, 0.0, 0.0, 1.0, // quat (x, y, z, w)
        0.0, 0.0, 0.0 // gyro
    };

    double motorData[24] = {};
    // Standing joint angles: [FR, FL, HR, HL]
    for (int leg = 0; leg < 4; ++leg) {
        motorData[leg * 3 + 0] = (leg % 2 == 0) ? -0.05 : 0.05;
        motorData[leg * 3 + 1] = -0.8;
        motorData[leg * 3 + 2] = 1.6;
        motorData[12 + leg * 3 + 0] = 0.0;
        motorData[12 + leg * 3 + 1] = 0.0;
        motorData[12 + leg * 3 + 2] = 0.0;
    }

    // Exercise TEST_A on an isolated controller because legacy hybrid commands are
    // torque-only; feeding their zero q_des back into this synthetic harness would
    // create an artificial joint-limit latch before the WBIC stages.
    {
        GaitCtrller legacy_controller(500.0, pidParam, true);
        legacy_controller.SetGaitType(0);
        legacy_controller.SetRobotMode(0);
        legacy_controller.SetRobotVel(target_vel);
        legacy_controller.SetStandingTestMode(wbic::StandingTestMode::TestALegacy);
        if (legacy_controller.IsWbicEnabled()) {
            std::cerr << "TEST_A_LEGACY unexpectedly enabled WBIC" << std::endl;
            return 6;
        }
        wbic::JointHybridCommand legacy_hybrid;
        double legacy_effort[12] = {};
        legacy_controller.TorqueCalculator(imuData, motorData, legacy_effort, &legacy_hybrid);
    }

    constexpr int kSimSteps = 1000; // 2.0 seconds at 500 Hz
    wbic::JointHybridCommand hybridCmd;
    double effort[12] = {};

    for (int step = 0; step < kSimSteps; ++step) {
        if (step == 250) {
            controller.SetStandingTestMode(wbic::StandingTestMode::TestCWbicLockBase);
        }
        if (step == 500) {
            controller.SetStandingTestMode(wbic::StandingTestMode::TestBWbicNormal);
            target_vel[0] = 0.1; // gentle forward walk
            controller.SetRobotVel(target_vel);
        }

        controller.TorqueCalculator(imuData, motorData, effort, &hybridCmd);
        const wbic::WbicStatus status = controller.LastWbicStatus();
        if (status != wbic::WbicStatus::Ok && status != wbic::WbicStatus::QpMaxIter) {
            std::cerr << "Invalid WBIC status at step " << step << ": "
                      << static_cast<int>(status) << std::endl;
            return 4;
        }

        // Verify outputs are finite
        for (int i = 0; i < 12; ++i) {
            if (!std::isfinite(effort[i])) {
                std::cerr << "Non-finite effort at step " << step << ", joint " << i << std::endl;
                return 1;
            }
            if (!std::isfinite(hybridCmd.tau_ff[i]) ||
                !std::isfinite(hybridCmd.q_des[i]) ||
                !std::isfinite(hybridCmd.qd_des[i])) {
                std::cerr << "Non-finite hybridCmd at step " << step << ", joint " << i << std::endl;
                return 2;
            }
            const double torque_limit = (i % 3 == 2) ? 36.0 : 24.0;
            if (std::abs(effort[i]) > torque_limit + 1e-6 ||
                std::abs(hybridCmd.tau_ff[i]) > torque_limit + 1e-6) {
                std::cerr << "Torque limit exceeded at step " << step << ": effort="
                          << effort[i] << ", tau_ff=" << hybridCmd.tau_ff[i] << std::endl;
                return 3;
            }
        }

        // Closed-loop state integration
        for (int i = 0; i < 12; ++i) {
            motorData[i] += 0.05 * (hybridCmd.q_des[i] - motorData[i]);
            motorData[12 + i] = hybridCmd.qd_des[i];
        }
    }

    if (controller.WbicFallbackCount() != 0) {
        std::cerr << "Unexpected WBIC fallbacks: " << controller.WbicFallbackCount() << std::endl;
        return 5;
    }

    std::cout << "[TEST] wbic_integration_smoke PASSED! (standing A/B/C exercised)" << std::endl;
    return 0;
}
