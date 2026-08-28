#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "CentroidalModel.hpp"
#include "CentroidalWrenchController.hpp"
#include "RobotModel.hpp"
#include "WbicController.hpp"

namespace {

constexpr double kFrequencyHz = 500.0;
constexpr double kDt = 1.0 / kFrequencyHz;
constexpr double kDurationSeconds = 20.0;
constexpr double kAnalysisStartSeconds = 5.0;
constexpr int kNumSteps = static_cast<int>(kDurationSeconds * kFrequencyHz) + 1;

struct Statistics {
    std::size_t count = 0;
    double mean = 0.0;
    double m2 = 0.0;
    double maximum = -std::numeric_limits<double>::infinity();

    void Add(double value)
    {
        ++count;
        const double delta = value - mean;
        mean += delta / static_cast<double>(count);
        const double delta_from_updated_mean = value - mean;
        m2 += delta * delta_from_updated_mean;
        maximum = std::max(maximum, value);
    }

    double Mean() const { return mean; }

    double StandardDeviation() const
    {
        return std::sqrt(m2 / static_cast<double>(count));
    }
};

Eigen::Matrix<double, 12, 1> Flatten(
    const std::array<Eigen::Vector3d, wbic::kNumLegs>& forces)
{
    Eigen::Matrix<double, 12, 1> result;
    for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
        result.segment<3>(3 * leg) = forces[static_cast<std::size_t>(leg)];
    }
    return result;
}

double PitchMoment(const Eigen::Matrix<double, 12, 1>& forces,
                   const std::array<Eigen::Vector3d, wbic::kNumLegs>& feet,
                   const Eigen::Vector3d& com)
{
    double moment_y = 0.0;
    for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
        moment_y += ((feet[static_cast<std::size_t>(leg)] - com)
                         .cross(forces.segment<3>(3 * leg)))
                        .y();
    }
    return moment_y;
}

void WriteHeader(std::ofstream& csv)
{
    csv << "time";
    constexpr const char* kLegNames[wbic::kNumLegs] = {"FR", "FL", "HR", "HL"};
    constexpr const char* kAxisNames[3] = {"x", "y", "z"};
    for (const char* prefix : {"Fr_des", "f_opt"}) {
        for (const char* leg : kLegNames) {
            for (const char* axis : kAxisNames) {
                csv << ',' << prefix << '_' << leg << '_' << axis;
            }
        }
    }
    csv << ",norm_f_opt_minus_Fr_des"
        << ",norm_diff_FR,norm_diff_FL,norm_diff_HR,norm_diff_HL"
        << ",norm_f_opt_delta"
        << ",Fz_total_Fr_des,Fz_total_f_opt"
        << ",My_Fr_des,My_f_opt\n";
}

void WriteSummaryLine(const char* name, const Statistics& stats, bool include_max)
{
    std::cout << name << ": mean=" << stats.Mean();
    if (include_max) std::cout << ", max=" << stats.maximum;
    else std::cout << ", std=" << stats.StandardDeviation();
    std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string csv_path = argc > 1
        ? argv[1]
        : "wbic_phase2_force_telemetry.csv";
    std::ofstream csv(csv_path, std::ios::out | std::ios::trunc);
    if (!csv) {
        std::cerr << "Cannot open telemetry CSV: " << csv_path << '\n';
        return 1;
    }
    csv << std::setprecision(17);
    WriteHeader(csv);

    RobotModel model;
    RobotModelConfig model_config;
    std::string model_error;
    if (!model.build(model_config, &model_error)) {
        std::cerr << "RobotModel build failed: " << model_error << '\n';
        return 2;
    }

    wbic::PayloadConfig payload;
    payload.enabled = true;
    payload.mass = 2.0;

    wbic::CentroidalWrenchController centroidal_controller;
    wbic::WbicController wbic_controller;

    wbic::WbicInput wbic_input;
    wbic_input.p_body = Eigen::Vector3d(0.0, 0.0, 0.30);
    wbic_input.R_body.setIdentity();
    wbic_input.p_body_des = wbic_input.p_body;
    wbic_input.R_body_des = wbic_input.R_body;
    wbic_input.standing_mode = true;
    wbic_input.payload_config = payload;
    wbic_input.contact.fill(true);
    wbic_input.phase.setOnes();
    for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
        wbic_input.q_joint_raw[3 * leg] = (leg % 2 == 0) ? -0.05 : 0.05;
        wbic_input.q_joint_raw[3 * leg + 1] = -0.8;
        wbic_input.q_joint_raw[3 * leg + 2] = 1.6;
    }

    model.setState(wbic_input.p_body,
                   Eigen::Quaterniond(wbic_input.R_body),
                   wbic_input.v_body_world,
                   wbic_input.omega_body_world,
                   wbic_input.q_joint_raw,
                   wbic_input.qd_joint_raw);
    model.updateKinematics();

    std::array<Eigen::Vector3d, wbic::kNumLegs> foot_positions{};
    for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
        foot_positions[static_cast<std::size_t>(leg)] = model.footPos(leg);
        wbic_input.p_foot_des[static_cast<std::size_t>(leg)] = model.footPos(leg);
    }

    const wbic::CentroidalState centroidal_state = wbic::ComputeCentroidalState(
        model.mass(), model.comPos(), model.comVel(), wbic_input.p_body,
        wbic_input.R_body, wbic_input.v_body_world, wbic_input.omega_body_world,
        payload);

    wbic::CentroidalWrenchInput centroidal_input;
    centroidal_input.mass = centroidal_state.mass;
    centroidal_input.com_world = centroidal_state.com_world;
    centroidal_input.com_des_world =
        wbic_input.p_body_des
        + wbic_input.R_body_des * centroidal_state.com_offset_body;
    centroidal_input.com_vel_world = centroidal_state.com_vel_world;
    centroidal_input.com_vel_des_world.setZero();
    centroidal_input.com_acc_des_world.setZero();
    centroidal_input.R_world_body = wbic_input.R_body;
    centroidal_input.R_des_world_body = wbic_input.R_body_des;
    centroidal_input.omega_world.setZero();
    centroidal_input.omega_des_world.setZero();
    centroidal_input.foot_pos_world = foot_positions;
    centroidal_input.contact.fill(true);

    Statistics force_difference;
    Statistics force_delta;
    Statistics fz_fr_des;
    Statistics fz_f_opt;
    Statistics my_fr_des;
    Statistics my_f_opt;
    Eigen::Matrix<double, 12, 1> previous_f_opt =
        Eigen::Matrix<double, 12, 1>::Zero();
    bool have_previous = false;

    for (int step = 0; step < kNumSteps; ++step) {
        const double time = step * kDt;
        wbic::CentroidalWrenchOutput centroidal_output;
        if (!centroidal_controller.Compute(centroidal_input, &centroidal_output)
            || !centroidal_output.valid) {
            std::cerr << "Centroidal QP failed at t=" << time << " s, status="
                      << centroidal_output.qp_status << '\n';
            return 3;
        }

        for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
            wbic_input.Fr_des[static_cast<std::size_t>(leg)] =
                centroidal_output.grf_world[static_cast<std::size_t>(leg)];
        }
        wbic_input.timestamp = time;
        wbic_input.sequence = static_cast<std::uint64_t>(step);

        wbic::WbicOutput wbic_output;
        const wbic::WbicStatus status = wbic_controller.Run(wbic_input, &wbic_output);
        if (status != wbic::WbicStatus::Ok && status != wbic::WbicStatus::QpMaxIter) {
            std::cerr << "WbicQp failed at t=" << time << " s, status="
                      << static_cast<int>(status) << '\n';
            return 4;
        }

        const Eigen::Matrix<double, 12, 1> fr_des = Flatten(centroidal_output.grf_world);
        const Eigen::Matrix<double, 12, 1>& f_opt = wbic_output.f_opt;
        const double difference_norm = (f_opt - fr_des).norm();
        std::array<double, wbic::kNumLegs> leg_difference_norm{};
        for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
            leg_difference_norm[static_cast<std::size_t>(leg)] =
                (f_opt.segment<3>(3 * leg) - fr_des.segment<3>(3 * leg)).norm();
        }
        const double delta_norm = have_previous ? (f_opt - previous_f_opt).norm() : 0.0;
        const double fz_des_total = fr_des[2] + fr_des[5] + fr_des[8] + fr_des[11];
        const double fz_opt_total = f_opt[2] + f_opt[5] + f_opt[8] + f_opt[11];
        const double my_des = PitchMoment(fr_des, foot_positions, centroidal_state.com_world);
        const double my_opt = PitchMoment(f_opt, foot_positions, centroidal_state.com_world);

        csv << time;
        for (int i = 0; i < 12; ++i) csv << ',' << fr_des[i];
        for (int i = 0; i < 12; ++i) csv << ',' << f_opt[i];
        csv << ',' << difference_norm;
        for (double leg_norm : leg_difference_norm) csv << ',' << leg_norm;
        csv << ',' << delta_norm
            << ',' << fz_des_total << ',' << fz_opt_total
            << ',' << my_des << ',' << my_opt << '\n';

        if (time >= kAnalysisStartSeconds) {
            force_difference.Add(difference_norm);
            force_delta.Add(delta_norm);
            fz_fr_des.Add(fz_des_total);
            fz_f_opt.Add(fz_opt_total);
            my_fr_des.Add(my_des);
            my_f_opt.Add(my_opt);
        }
        previous_f_opt = f_opt;
        have_previous = true;
    }

    csv.close();
    std::cout << std::setprecision(12)
              << "configuration: duration_s=" << kDurationSeconds
              << ", frequency_hz=" << kFrequencyHz
              << ", payload_kg=" << payload.mass
              << ", centroidal_on=1, payload_aware_on=1, lock_base_on=1\n"
              << "analysis: t >= " << kAnalysisStartSeconds
              << " s, samples=" << force_difference.count << '\n';
    WriteSummaryLine("norm_f_opt_minus_Fr_des_N", force_difference, true);
    WriteSummaryLine("norm_f_opt_delta_N", force_delta, true);
    WriteSummaryLine("Fz_total_Fr_des_N", fz_fr_des, false);
    WriteSummaryLine("Fz_total_f_opt_N", fz_f_opt, false);
    WriteSummaryLine("My_Fr_des_Nm", my_fr_des, false);
    WriteSummaryLine("My_f_opt_Nm", my_f_opt, false);
    std::cout << "telemetry_csv=" << csv_path << '\n';
    return 0;
}
