#pragma once

#ifndef CMPC_LOGGER_HPP_
#define CMPC_LOGGER_HPP_

#include <iostream>
#include <fstream>
#include <sstream>
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
                  << "kf_contact_fl,kf_contact_fr,kf_contact_hl,kf_contact_hr,"
                  // 3. Adaptive Mass & CoM
                  << "est_mass_raw,est_mass_filtered,est_com_x,est_com_y,est_com_z,total_support_fz,"
                  // 4. MPC & Actual Foot Forces (Z component)
                  << "f_mpc_fr_z,f_mpc_fl_z,f_mpc_hr_z,f_mpc_hl_z,"
                  << "f_act_fr_z,f_act_fl_z,f_act_hr_z,f_act_hl_z,"
                  // 5. Timers
                  << "t_est_ms,t_mpc_ms,t_total_ms\n";
    }

    void Log(double time_ms, const CmpcTelemetryData& telem) {
        if (!is_open_) return;
        record_count_++;

        char line_buf[1024];
        int len = snprintf(line_buf, sizeof(line_buf),
            "%.3f,%lu,"
            // Commands
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
            // KF Positions & Velocities
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
            // KF Euler & Omega & Acc
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
            // KF Contacts
            "%.2f,%.2f,%.2f,%.2f,"
            // Mass & CoM
            "%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,"
            // MPC & Actual Forces (Z)
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
            // Timers
            "%.3f,%.3f,%.3f\n",
            time_ms, (unsigned long)record_count_,
            telem.cmd_vx, telem.cmd_vy, telem.cmd_yaw_rate,
            telem.des_vx, telem.des_vy, telem.des_yaw_rate,
            telem.kf_pos[0], telem.kf_pos[1], telem.kf_pos[2],
            telem.kf_vel_world[0], telem.kf_vel_world[1], telem.kf_vel_world[2],
            telem.kf_vel_body[0], telem.kf_vel_body[1], telem.kf_vel_body[2],
            telem.kf_rpy[0], telem.kf_rpy[1], telem.kf_rpy[2],
            telem.kf_omega_body[0], telem.kf_omega_body[1], telem.kf_omega_body[2],
            telem.kf_acc_body[0], telem.kf_acc_body[1], telem.kf_acc_body[2],
            telem.kf_contact_prob[1], telem.kf_contact_prob[0], telem.kf_contact_prob[3], telem.kf_contact_prob[2], // FL, FR, HL, HR
            telem.est_mass_raw, telem.est_mass_filtered,
            telem.est_com_body[0], telem.est_com_body[1], telem.est_com_body[2],
            telem.total_support_force_z,
            telem.f_mpc_des_world[0][2], telem.f_mpc_des_world[1][2], telem.f_mpc_des_world[2][2], telem.f_mpc_des_world[3][2],
            telem.f_act_est_world[0][2], telem.f_act_est_world[1][2], telem.f_act_est_world[2][2], telem.f_act_est_world[3][2],
            telem.t_est_ms, telem.t_mpc_ms, telem.t_total_ms
        );

        if (len > 0) {
            log_file_.write(line_buf, len);
        }
    }
};

#endif // CMPC_LOGGER_HPP_
