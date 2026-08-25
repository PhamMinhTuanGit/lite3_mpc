#pragma once

#ifndef CMPC_LOGGER_HPP_
#define CMPC_LOGGER_HPP_

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>
#include "MPC_Ctrl/CmpcTelemetry.h"

class CmpcLogger {
private:
    std::ofstream log_file_;
    std::string file_path_;
    bool is_open_ = false;
    uint64_t record_count_ = 0;

public:
    CmpcLogger() = default;
    ~CmpcLogger() {
        Close();
    }

    void Init() {
        Close();
        struct stat st = {0};
        if (stat("../data", &st) == -1) {
            mkdir("../data", 0777);
        }

        std::time_t now = std::time(nullptr);
        std::tm* local_time = std::localtime(&now);
        char time_buf[64];
        std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", local_time);

        file_path_ = "../data/cmpc_telemetry_" + std::string(time_buf) + ".csv";
        log_file_.open(file_path_, std::ios::out);

        if (log_file_.is_open()) {
            is_open_ = true;
            record_count_ = 0;
            WriteHeader();
            std::cout << "[CmpcLogger] Recording CMPC & Linear KF telemetry to: " << file_path_ << std::endl;
        } else {
            std::cerr << "[CmpcLogger] Warning: Could not open log file: " << file_path_ << std::endl;
        }
    }

    void Close() {
        if (is_open_ && log_file_.is_open()) {
            log_file_.flush();
            log_file_.close();
            is_open_ = false;
            std::cout << "[CmpcLogger] Saved " << record_count_ << " telemetry frames to " << file_path_ << std::endl;
        }
    }

    void WriteHeader() {
        if (!is_open_) return;
        log_file_ << "timestamp_ms,record_count,"
                  // 1. Reference & Filtered Commands
                  << "cmd_vx,cmd_vy,cmd_yaw_rate,des_vx,des_vy,des_yaw_rate,"
                  // 2. Linear KF State Estimation
                  << "kf_pos_x,kf_pos_y,kf_pos_z,"
                  << "kf_vel_world_x,kf_vel_world_y,kf_vel_world_z,"
                  << "kf_vel_body_x,kf_vel_body_y,kf_vel_body_z,"
                  << "kf_roll,kf_pitch,kf_yaw,"
                  << "kf_omega_body_x,kf_omega_body_y,kf_omega_body_z,"
                  << "kf_acc_body_x,kf_acc_body_y,kf_acc_body_z,"
                  << "kf_contact_fr,kf_contact_fl,kf_contact_hr,kf_contact_hl,"
                  // 3. Adaptive Mass & CoM
                  << "est_mass_raw,est_mass_filtered,est_com_x,est_com_y,est_com_z,total_support_fz,"
                  // 4. MPC & Actual Foot Forces (Z component)
                  << "f_mpc_fr_z,f_mpc_fl_z,f_mpc_hr_z,f_mpc_hl_z,"
                  << "f_act_fr_z,f_act_fl_z,f_act_hr_z,f_act_hl_z,"
                  << "gmo_valid,gmo_initialized,gmo_ready,gmo_invalid_reason,"
                  << "gmo_samples_since_reset,gmo_reset_count,"
                  << "grf_valid,grf_ready,grf_invalid_reason,";
        for (int i = 0; i < 18; ++i) log_file_ << "gmo_p_" << i << ',';
        for (int i = 0; i < 18; ++i) log_file_ << "gmo_p_hat_" << i << ',';
        for (int i = 0; i < 18; ++i) log_file_ << "gmo_residual_" << i << ',';
        const char* legs[4] = {"fr", "fl", "hr", "hl"};
        const char* axes[3] = {"x", "y", "z"};
        for (int leg = 0; leg < 4; ++leg) {
            for (int axis = 0; axis < 3; ++axis)
                log_file_ << "gmo_force_raw_" << legs[leg] << '_' << axes[axis] << ',';
            for (int axis = 0; axis < 3; ++axis)
                log_file_ << "gmo_force_" << legs[leg] << '_' << axes[axis] << ',';
            log_file_ << "gmo_force_" << legs[leg] << "_norm,"
                      << "grf_leg_valid_" << legs[leg] << ','
                      << "grf_jacobian_quality_" << legs[leg] << ',';
        }
        for (int leg = 0; leg < 4; ++leg)
            log_file_ << "scheduled_contact_" << legs[leg] << ',';
        log_file_ << "gt_valid,";
        for (int leg = 0; leg < 4; ++leg) {
            log_file_ << "gt_contact_" << legs[leg] << ',';
            for (int axis = 0; axis < 3; ++axis)
                log_file_ << "gt_force_" << legs[leg] << '_' << axes[axis] << ',';
        }
        for (int leg = 0; leg < 4; ++leg) log_file_ << "gmo_contact_binary_" << legs[leg] << ',';
        for (int leg = 0; leg < 4; ++leg) log_file_ << "lost_contact_time_ms_" << legs[leg] << ',';
        for (int leg = 0; leg < 4; ++leg) log_file_ << "stance_force_scale_" << legs[leg] << ',';
        for (int leg = 0; leg < 4; ++leg) log_file_ << "q_abad_" << legs[leg] << ',';
        for (int leg = 0; leg < 4; ++leg) log_file_ << "tau_abad_cmd_" << legs[leg] << ',';
        log_file_ << "t_est_ms,t_gmo_ms,t_grf_ms,t_evidence_p99_ms,"
                  << "t_mpc_ms,t_total_ms\n";
    }

    void Log(double time_ms, const CmpcTelemetryData& telem) {
        if (!is_open_) return;
        record_count_++;

        std::ostringstream line;
        line << std::fixed << std::setprecision(4)
             << time_ms << ',' << record_count_ << ','
             << telem.cmd_vx << ',' << telem.cmd_vy << ',' << telem.cmd_yaw_rate << ','
             << telem.des_vx << ',' << telem.des_vy << ',' << telem.des_yaw_rate << ',';
        for (float value : telem.kf_pos) line << value << ',';
        for (float value : telem.kf_vel_world) line << value << ',';
        for (float value : telem.kf_vel_body) line << value << ',';
        for (float value : telem.kf_rpy) line << value << ',';
        for (float value : telem.kf_omega_body) line << value << ',';
        for (float value : telem.kf_acc_body) line << value << ',';
        for (float value : telem.kf_contact_prob) line << value << ',';
        line
             << telem.est_mass_raw << ',' << telem.est_mass_filtered << ',';
        for (float value : telem.est_com_body) line << value << ',';
        line << telem.total_support_force_z << ',';
        for (int leg = 0; leg < 4; ++leg) line << telem.f_mpc_des_world[leg][2] << ',';
        for (int leg = 0; leg < 4; ++leg) line << telem.f_act_est_world[leg][2] << ',';
        line << static_cast<int>(telem.gmo_valid) << ','
             << static_cast<int>(telem.gmo_initialized) << ','
             << static_cast<int>(telem.gmo_ready) << ','
             << static_cast<int>(telem.gmo_invalid_reason) << ','
             << telem.gmo_samples_since_reset << ','
             << telem.gmo_reset_count << ','
             << static_cast<int>(telem.grf_valid) << ','
             << static_cast<int>(telem.grf_ready) << ','
             << static_cast<int>(telem.grf_invalid_reason) << ',';
        for (float value : telem.gmo_momentum) line << value << ',';
        for (float value : telem.gmo_momentum_hat) line << value << ',';
        for (float value : telem.gmo_residual) line << value << ',';
        for (int leg = 0; leg < 4; ++leg) {
            for (float value : telem.gmo_force_raw_world[leg]) line << value << ',';
            for (float value : telem.gmo_force_world[leg]) line << value << ',';
            line << telem.gmo_force_norm[leg] << ','
                 << static_cast<int>(telem.grf_leg_valid[leg]) << ','
                 << telem.grf_jacobian_quality[leg] << ',';
        }
        for (float value : telem.scheduled_contact) line << value << ',';
        line << static_cast<int>(telem.gt_valid) << ',';
        for (int leg = 0; leg < 4; ++leg) {
            line << telem.gt_contact[leg] << ',';
            for (float value : telem.gt_force_world[leg]) line << value << ',';
        }
        for (int leg = 0; leg < 4; ++leg) line << static_cast<int>(telem.gmo_contact_binary[leg]) << ',';
        for (int leg = 0; leg < 4; ++leg) line << telem.lost_contact_time_ms[leg] << ',';
        for (int leg = 0; leg < 4; ++leg) line << telem.stance_force_scale[leg] << ',';
        for (int leg = 0; leg < 4; ++leg) line << telem.q_abad[leg] << ',';
        for (int leg = 0; leg < 4; ++leg) line << telem.tau_abad_cmd[leg] << ',';
        line << telem.t_est_ms << ',' << telem.t_gmo_ms << ','
             << telem.t_grf_ms << ',' << telem.t_evidence_p99_ms << ','
             << telem.t_mpc_ms << ','
             << telem.t_total_ms << '\n';
        log_file_ << line.str();
    }
};

#endif // CMPC_LOGGER_HPP_
