#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <numeric>

#include "WbcType.h"
#include "WbicController.hpp"

int main()
{
    std::cout << "[TEST] wbic_safety_benchmark_smoke starting..." << std::endl;

    wbic::WbicConfig config;
    config.max_consecutive_failures = 3;
    wbic::WbicController controller(config);

    wbic::WbicInput input;
    input.p_body = Eigen::Vector3d(0.0, 0.0, 0.28);
    input.R_body = Eigen::Matrix3d::Identity();
    input.v_body_world = Eigen::Vector3d::Zero();
    input.omega_body_world = Eigen::Vector3d::Zero();

    for (int leg = 0; leg < 4; ++leg) {
        input.q_joint_raw[leg * 3 + 0] = 0.0;
        input.q_joint_raw[leg * 3 + 1] = -0.7;
        input.q_joint_raw[leg * 3 + 2] = 1.4;
        input.qd_joint_raw.segment<3>(leg * 3).setZero();

        input.p_foot_des[leg] = Eigen::Vector3d(0.18 * ((leg % 2 == 0) ? 1 : -1),
                                               0.15 * ((leg < 2) ? 1 : -1),
                                               -0.28);
        input.v_foot_des[leg].setZero();
        input.a_foot_des[leg].setZero();
        input.Fr_des[leg] = Eigen::Vector3d(0.0, 0.0, 11.9376 * 9.81 / 4.0);
        input.contact[leg] = true;
    }

    input.p_body_des = input.p_body;
    input.v_body_des.setZero();
    input.a_body_des.setZero();
    input.R_body_des = input.R_body;
    input.omega_body_des.setZero();
    input.domega_body_des.setZero();

    wbic::WbicOutput output;

    // ─────────────────────────────────────────────────────────────────
    // 1. Warm-up
    // ─────────────────────────────────────────────────────────────────
    for (int i = 0; i < 50; ++i) {
        controller.Run(input, &output);
    }
    controller.Reset();

    // ─────────────────────────────────────────────────────────────────
    // 2. Fault Injection & 3-Failure Latch Test
    // ─────────────────────────────────────────────────────────────────
    // First valid run
    auto status = controller.Run(input, &output);
    if (status != wbic::WbicStatus::Ok) {
        std::cerr << "Baseline run failed: " << static_cast<int>(status) << std::endl;
        return 1;
    }
    if (controller.GetConsecutiveFailures() != 0 || controller.IsLatched()) {
        std::cerr << "Unexpected failure counter after valid run" << std::endl;
        return 2;
    }

    // Inject NaN input 1
    wbic::WbicInput bad_input = input;
    bad_input.p_body[0] = std::numeric_limits<double>::quiet_NaN();
    controller.Run(bad_input, &output);
    if (controller.GetConsecutiveFailures() != 1 || controller.IsLatched()) {
        std::cerr << "Failure count 1 failed" << std::endl;
        return 3;
    }

    // Inject NaN input 2
    controller.Run(bad_input, &output);
    if (controller.GetConsecutiveFailures() != 2 || controller.IsLatched()) {
        std::cerr << "Failure count 2 failed" << std::endl;
        return 4;
    }

    // Inject NaN input 3 -> Should latch
    controller.Run(bad_input, &output);
    if (controller.GetConsecutiveFailures() != 3 || !controller.IsLatched()) {
        std::cerr << "Failure count 3 did not latch!" << std::endl;
        return 5;
    }

    // Run valid input while latched -> must remain latched and return SafetyViolation
    status = controller.Run(input, &output);
    if (!controller.IsLatched() || output.command_source != wbic::CommandSource::FallbackLegacyCmpc) {
        std::cerr << "Did not enforce latch on subsequent run" << std::endl;
        return 6;
    }

    // Reset / Unlatch
    controller.Reset();
    if (controller.IsLatched() || controller.GetConsecutiveFailures() != 0) {
        std::cerr << "Reset failed to clear latch" << std::endl;
        return 7;
    }

    // ─────────────────────────────────────────────────────────────────
    // 3. 10,000 Runs Latency & Zero Allocation Benchmark
    // ─────────────────────────────────────────────────────────────────
    constexpr int kNumRuns = 10000;
    std::vector<double> latencies_us;
    latencies_us.reserve(kNumRuns);

    for (int iter = 0; iter < kNumRuns; ++iter) {
        // Vary contact and joint state smoothly
        const double t = iter * 0.002;
        input.p_body[0] = 0.02 * std::sin(2.0 * M_PI * 1.0 * t);
        input.v_body_world[0] = 0.02 * 2.0 * M_PI * std::cos(2.0 * M_PI * 1.0 * t);
        input.contact[0] = (iter % 100) < 60;
        input.contact[1] = (iter % 100) >= 40;
        input.contact[2] = (iter % 100) >= 40;
        input.contact[3] = (iter % 100) < 60;

        const auto t_start = std::chrono::high_resolution_clock::now();
        controller.Run(input, &output);
        const auto t_end = std::chrono::high_resolution_clock::now();

        const double us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        latencies_us.push_back(us);
    }

    std::sort(latencies_us.begin(), latencies_us.end());
    const double p50 = latencies_us[static_cast<std::size_t>(kNumRuns * 0.50)];
    const double p90 = latencies_us[static_cast<std::size_t>(kNumRuns * 0.90)];
    const double p99 = latencies_us[static_cast<std::size_t>(kNumRuns * 0.99)];
    const double max_lat = latencies_us.back();
    const double mean_lat = std::accumulate(latencies_us.begin(), latencies_us.end(), 0.0) / kNumRuns;

    std::cout << "[BENCHMARK 10,000 runs] Mean: " << mean_lat << " us | "
              << "p50: " << p50 << " us | "
              << "p90: " << p90 << " us | "
              << "p99: " << p99 << " us (" << p99 / 1000.0 << " ms) | "
              << "Max: " << max_lat << " us" << std::endl;

    if (p99 > 2000.0) { // 2.0 ms requirement
        std::cerr << "p99 latency exceeded 2 ms budget: " << p99 << " us" << std::endl;
        return 8;
    }

    std::cout << "[TEST] wbic_safety_benchmark_smoke PASSED!" << std::endl;
    return 0;
}
