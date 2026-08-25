/**
 * @file data_streaming.hpp
 * @brief
 * @author mazunwang
 * @version 1.0
 * @date 2024-04-28
 *
 * @copyright Copyright (c) 2024  DeepRobotics
 *
 */
#ifndef DATA_STREAMING_HPP_
#define DATA_STREAMING_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sstream>
#include <fstream>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <csignal>
#include <mutex>
#include "json.hpp"
#include "MPC_Ctrl/CmpcTelemetry.h"

#define LOCAL_PORT 1021

class DataStreaming{
private:
    int sockfd_;
    sockaddr_in remote_addr_;
    std::mutex data_mutex_;

    FILE *fp_;
    std::string file_name_;
    std::ofstream file_;
    std::vector<float> scope_data_;

    nlohmann::json data_json_;
    int run_cnt_ = 0;
    const bool enable_online_plot_;
    const bool enable_data_record_;

    double GetTimestampMs(){
        static timespec startup_timestamp;
        timespec now_timestamp;
        if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
            clock_gettime(CLOCK_MONOTONIC,&startup_timestamp);
        }
        clock_gettime(CLOCK_MONOTONIC,&now_timestamp);
        return (now_timestamp.tv_sec-startup_timestamp.tv_sec)*1e3 + (now_timestamp.tv_nsec-startup_timestamp.tv_nsec)/1e6;
    }


public:
    DataStreaming(bool enable_online_plot=true, bool enable_data_record=true, const std::string& ip="127.0.0.1", int port=9870):
    enable_online_plot_(enable_online_plot),
    enable_data_record_(enable_data_record){
        if(enable_online_plot_){
            std::cout << "You can plot data online on plotjuggler" << std::endl;
            sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&remote_addr_, 0, sizeof(remote_addr_));
            remote_addr_.sin_family = AF_INET;
            remote_addr_.sin_port = htons(port);
            remote_addr_.sin_addr.s_addr = inet_addr(ip.c_str());

            // 设置本地地址和端口号
            struct sockaddr_in local_addr;
            std::memset(&local_addr, 0, sizeof(local_addr));
            local_addr.sin_family = AF_INET;
            local_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 绑定到所有网络接口
            local_addr.sin_port = htons(LOCAL_PORT); // 设置本地端口号

            // 绑定 socket 到本地地址和端口号
            if (bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) == -1) {
                std::cerr << "Failed to bind socket" << std::endl;
            }
            if (sockfd_ < 0) {
                perror("robot sender socket creation failed");
            }
        }

        if(enable_data_record_){
            std::time_t now = std::time(nullptr);
            std::tm* localTime = std::localtime(&now);
            char buffer[100];
            std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", localTime);
            file_name_ = "../data/";

            file_name_.append(std::string(buffer));
            file_name_.append(".csv");

            std::cout << "data is recorded in " << file_name_ << std::endl;
            file_.open(file_name_);
            file_.close();

            char username[256];
            if (getlogin_r(username, sizeof(username)) == 0) {
                std::cout << "Username: " << username << std::endl;
            } else {
                std::cerr << "Failed to get username" << std::endl;
            }

            std::string command = "chown " + std::string(username) + ":" + std::string(username) + " " + file_name_;
            int result = system(command.c_str());
            if (result == 0) {
                std::cout << "File permissions changed to 0666 (read and write for all users)" << std::endl;
            } else {
                std::cerr << "Failed to change file permissions" << std::endl;
            }

            file_.open(file_name_);
            if (!file_.is_open()) {
                std::cerr << "Failed to open file " << file_name_ << std::endl;
            }
        }
        scope_data_.resize(10, 0);
        InitJsonSchema();
    }
    ~DataStreaming(){}

    void InitJsonSchema() {
        data_json_["t_r"] = 0.0f;
        data_json_["t_i"] = 0.0f;
        data_json_["state"]["current_state"] = 0;
        data_json_["user_command"]["target_mode"] = 0;

        // Pre-populate CMPC & Linear KF telemetry keys so PlotJuggler receives the full schema from packet 0
        data_json_["cmpc"]["cmd_vx"] = 0.0f;
        data_json_["cmpc"]["cmd_vy"] = 0.0f;
        data_json_["cmpc"]["cmd_yaw_rate"] = 0.0f;
        data_json_["cmpc"]["des_vx"] = 0.0f;
        data_json_["cmpc"]["des_vy"] = 0.0f;
        data_json_["cmpc"]["des_yaw_rate"] = 0.0f;

        data_json_["cmpc"]["kf_pos_x"] = 0.0f;
        data_json_["cmpc"]["kf_pos_y"] = 0.0f;
        data_json_["cmpc"]["kf_pos_z"] = 0.29f;

        data_json_["cmpc"]["kf_vel_world_x"] = 0.0f;
        data_json_["cmpc"]["kf_vel_world_y"] = 0.0f;
        data_json_["cmpc"]["kf_vel_world_z"] = 0.0f;

        data_json_["cmpc"]["kf_vel_body_x"] = 0.0f;
        data_json_["cmpc"]["kf_vel_body_y"] = 0.0f;
        data_json_["cmpc"]["kf_vel_body_z"] = 0.0f;

        data_json_["cmpc"]["kf_roll"] = 0.0f;
        data_json_["cmpc"]["kf_pitch"] = 0.0f;
        data_json_["cmpc"]["kf_yaw"] = 0.0f;

        data_json_["cmpc"]["kf_omega_x"] = 0.0f;
        data_json_["cmpc"]["kf_omega_y"] = 0.0f;
        data_json_["cmpc"]["kf_omega_z"] = 0.0f;

        data_json_["cmpc"]["est_mass_raw"] = 11.94f;
        data_json_["cmpc"]["est_mass_filtered"] = 11.94f;
        data_json_["cmpc"]["est_com_x"] = 0.0f;
        data_json_["cmpc"]["est_com_y"] = 0.0f;
        data_json_["cmpc"]["est_com_z"] = 0.0f;
        data_json_["cmpc"]["total_support_fz"] = 0.0f;

        data_json_["cmpc"]["t_est_ms"] = 0.0f;
        data_json_["cmpc"]["gmo_valid"] = false;
        data_json_["cmpc"]["gmo_initialized"] = false;
        data_json_["cmpc"]["gmo_ready"] = false;
        data_json_["cmpc"]["gmo_invalid_reason"] = 0;
        data_json_["cmpc"]["gmo_samples_since_reset"] = 0;
        data_json_["cmpc"]["gmo_reset_count"] = 0;
        data_json_["cmpc"]["grf_valid"] = false;
        data_json_["cmpc"]["grf_ready"] = false;
        data_json_["cmpc"]["grf_invalid_reason"] = 0;
        data_json_["cmpc"]["grf_leg_valid"] = std::vector<int>(4, 0);
        data_json_["cmpc"]["grf_jacobian_quality"] = std::vector<float>(4, 0.0f);
        data_json_["cmpc"]["gmo_residual"] = std::vector<float>(18, 0.0f);
        data_json_["cmpc"]["gmo_force_raw_world"] = std::vector<float>(12, 0.0f);
        data_json_["cmpc"]["gmo_force_world"] = std::vector<float>(12, 0.0f);
        data_json_["cmpc"]["scheduled_contact"] = std::vector<float>(4, 0.0f);
        data_json_["cmpc"]["gt_valid"] = false;
        data_json_["cmpc"]["gt_contact"] = std::vector<float>(4, 0.0f);
        data_json_["cmpc"]["gt_force_world"] = std::vector<float>(12, 0.0f);
        data_json_["cmpc"]["t_gmo_ms"] = 0.0f;
        data_json_["cmpc"]["t_grf_ms"] = 0.0f;
        data_json_["cmpc"]["t_evidence_p99_ms"] = 0.0f;
        data_json_["cmpc"]["t_mpc_ms"] = 0.0f;
        data_json_["cmpc"]["t_total_ms"] = 0.0f;
    }

    void SendData(){
        std::lock_guard<std::mutex> lock(data_mutex_);
        ++run_cnt_;
        float ts = GetTimestampMs();
        data_json_["t_r"] = ts/1000.;
        data_json_["scope"] = scope_data_;
        if(enable_data_record_){
            if(run_cnt_ == 1) WriteItemNameToCsv();
            WriteDataToCsv();
        }
        if(enable_online_plot_){
            std::string json_str = data_json_.dump();
            int nbytes = sendto(sockfd_, json_str.c_str(), json_str.length(), 0,
                        (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
            if(nbytes < 0){
                std::cerr << "Error: Could not send message" << std::endl;
            }
        }
        // return nbytes;
    }

    void InsertCmpcTelemetry(const CmpcTelemetryData& telem){
        std::lock_guard<std::mutex> lock(data_mutex_);
        data_json_["cmpc"]["cmd_vx"] = telem.cmd_vx;
        data_json_["cmpc"]["cmd_vy"] = telem.cmd_vy;
        data_json_["cmpc"]["cmd_yaw_rate"] = telem.cmd_yaw_rate;
        data_json_["cmpc"]["des_vx"] = telem.des_vx;
        data_json_["cmpc"]["des_vy"] = telem.des_vy;
        data_json_["cmpc"]["des_yaw_rate"] = telem.des_yaw_rate;

        data_json_["cmpc"]["kf_pos_x"] = telem.kf_pos[0];
        data_json_["cmpc"]["kf_pos_y"] = telem.kf_pos[1];
        data_json_["cmpc"]["kf_pos_z"] = telem.kf_pos[2];

        data_json_["cmpc"]["kf_vel_world_x"] = telem.kf_vel_world[0];
        data_json_["cmpc"]["kf_vel_world_y"] = telem.kf_vel_world[1];
        data_json_["cmpc"]["kf_vel_world_z"] = telem.kf_vel_world[2];

        data_json_["cmpc"]["kf_vel_body_x"] = telem.kf_vel_body[0];
        data_json_["cmpc"]["kf_vel_body_y"] = telem.kf_vel_body[1];
        data_json_["cmpc"]["kf_vel_body_z"] = telem.kf_vel_body[2];

        data_json_["cmpc"]["kf_roll"] = telem.kf_rpy[0];
        data_json_["cmpc"]["kf_pitch"] = telem.kf_rpy[1];
        data_json_["cmpc"]["kf_yaw"] = telem.kf_rpy[2];

        data_json_["cmpc"]["kf_omega_x"] = telem.kf_omega_body[0];
        data_json_["cmpc"]["kf_omega_y"] = telem.kf_omega_body[1];
        data_json_["cmpc"]["kf_omega_z"] = telem.kf_omega_body[2];

        data_json_["cmpc"]["est_mass_raw"] = telem.est_mass_raw;
        data_json_["cmpc"]["est_mass_filtered"] = telem.est_mass_filtered;
        data_json_["cmpc"]["est_com_x"] = telem.est_com_body[0];
        data_json_["cmpc"]["est_com_y"] = telem.est_com_body[1];
        data_json_["cmpc"]["est_com_z"] = telem.est_com_body[2];
        data_json_["cmpc"]["total_support_fz"] = telem.total_support_force_z;

        data_json_["cmpc"]["gmo_valid"] = telem.gmo_valid != 0;
        data_json_["cmpc"]["gmo_initialized"] = telem.gmo_initialized != 0;
        data_json_["cmpc"]["gmo_ready"] = telem.gmo_ready != 0;
        data_json_["cmpc"]["gmo_invalid_reason"] = telem.gmo_invalid_reason;
        data_json_["cmpc"]["gmo_samples_since_reset"] = telem.gmo_samples_since_reset;
        data_json_["cmpc"]["gmo_reset_count"] = telem.gmo_reset_count;
        data_json_["cmpc"]["grf_valid"] = telem.grf_valid != 0;
        data_json_["cmpc"]["grf_ready"] = telem.grf_ready != 0;
        data_json_["cmpc"]["grf_invalid_reason"] = telem.grf_invalid_reason;
        data_json_["cmpc"]["gmo_residual"] =
            std::vector<float>(telem.gmo_residual, telem.gmo_residual + 18);
        std::vector<float> gmo_force(12);
        std::vector<float> gmo_force_raw(12);
        std::vector<float> gt_force(12);
        std::vector<int> grf_leg_valid(4);
        std::vector<float> grf_jacobian_quality(4);
        for (int leg = 0; leg < 4; ++leg) {
            grf_leg_valid[leg] = telem.grf_leg_valid[leg];
            grf_jacobian_quality[leg] = telem.grf_jacobian_quality[leg];
            for (int axis = 0; axis < 3; ++axis) {
                gmo_force_raw[leg * 3 + axis] =
                    telem.gmo_force_raw_world[leg][axis];
                gmo_force[leg * 3 + axis] = telem.gmo_force_world[leg][axis];
                gt_force[leg * 3 + axis] = telem.gt_force_world[leg][axis];
            }
        }
        data_json_["cmpc"]["gmo_force_raw_world"] = gmo_force_raw;
        data_json_["cmpc"]["gmo_force_world"] = gmo_force;
        data_json_["cmpc"]["grf_leg_valid"] = grf_leg_valid;
        data_json_["cmpc"]["grf_jacobian_quality"] = grf_jacobian_quality;
        data_json_["cmpc"]["scheduled_contact"] =
            std::vector<float>(telem.scheduled_contact, telem.scheduled_contact + 4);
        data_json_["cmpc"]["gt_valid"] = telem.gt_valid != 0;
        data_json_["cmpc"]["gt_contact"] =
            std::vector<float>(telem.gt_contact, telem.gt_contact + 4);
        data_json_["cmpc"]["gt_force_world"] = gt_force;

        data_json_["cmpc"]["t_est_ms"] = telem.t_est_ms;
        data_json_["cmpc"]["t_gmo_ms"] = telem.t_gmo_ms;
        data_json_["cmpc"]["t_grf_ms"] = telem.t_grf_ms;
        data_json_["cmpc"]["t_evidence_p99_ms"] = telem.t_evidence_p99_ms;
        data_json_["cmpc"]["t_mpc_ms"] = telem.t_mpc_ms;
        data_json_["cmpc"]["t_total_ms"] = telem.t_total_ms;
    }

    void InsertInterfaceTime(float data){
        data_json_["t_i"] = data;
    }

    void InsertJointData(const std::string &name, float data){
        data_json_["joint"][name] = data;
    }

    void InsertJointData(const std::string &name,const VecXf& data){
        std::vector<float> vec(data.data(), data.data()+data.size());
        data_json_["joint"][name] = vec;
    }

    void InsertImuData(const std::string &name, float data){
        data_json_["imu"][name] = data;
    }

    void InsertImuData(const std::string &name,const Vec3f& data){
        std::vector<float> vec(data.data(), data.data()+data.size());
        data_json_["imu"][name] = vec;
    }

    void InsertCommandData(const std::string &name, float data){
        data_json_["user_command"][name] = data;
    }

    void InsertStateData(const std::string &name, float data){
        data_json_["state"][name] = data;
    }

    void InsertScopeData(int index, float data){
        scope_data_[index] = data;
    }

    void WriteItemNameToCsv(){
        file_ << "run_cnt" << "," << "t_r" << "," << "t_i" << ",";
        for(int i=0;i<data_json_["joint"].at("q").size();++i){
            file_ << "q"+std::to_string(i) << ",";
        }
        for(int i=0;i<data_json_["joint"].at("dq").size();++i){
            file_ << "dq"+std::to_string(i) << ",";
        }
        for(int i=0;i<data_json_["joint"].at("tau").size();++i){
            file_ << "tau"+std::to_string(i) << ",";
        }
        for(int i=0;i<data_json_["joint"].at("q_cmd").size();++i){
            file_ << "q_cmd"+std::to_string(i) << ",";
        }
        for(int i=0;i<data_json_["joint"].at("tau_ff").size();++i){
            file_ << "tau_ff"+std::to_string(i) << ",";
        }
        for(int i=0;i<data_json_["imu"].at("rpy").size();++i){
            file_ << "rpy"+std::to_string(i) << ",";
        }
        for(int i=0;i<data_json_["imu"].at("acc").size();++i){
            file_ << "acc"+std::to_string(i) << ",";
        }
        for(int i=0;i<data_json_["imu"].at("omg").size();++i){
            file_ << "omg"+std::to_string(i) << ",";
        }
        file_ << "state" << ",";
        for(int i=0;i<scope_data_.size();++i){
            file_ << "scope"+std::to_string(i) << ",";
        }
        file_ << "\n";
    }


    void WriteDataToCsv(){
        std::ostringstream oss;
        // oss << std::fixed << std::setprecision(3);
        oss << run_cnt_ << "," << data_json_["t_r"] << "," << data_json_["t_i"] << ",";
        for(int i=0;i<data_json_["joint"].at("q").size();++i){
            oss << data_json_["joint"].at("q")[i] << ",";
        }
        for(int i=0;i<data_json_["joint"].at("dq").size();++i){
            oss << data_json_["joint"].at("dq")[i] << ",";
        }
        for(int i=0;i<data_json_["joint"].at("tau").size();++i){
            oss << data_json_["joint"].at("tau")[i] << ",";
        }
        for(int i=0;i<data_json_["joint"].at("q_cmd").size();++i){
            oss << data_json_["joint"].at("q_cmd")[i] << ",";
        }
        for(int i=0;i<data_json_["joint"].at("tau_ff").size();++i){
            oss << data_json_["joint"].at("tau_ff")[i] << ",";
        }
        for(int i=0;i<data_json_["imu"].at("rpy").size();++i){
            oss << data_json_["imu"].at("rpy")[i] << ",";
        }
        for(int i=0;i<data_json_["imu"].at("acc").size();++i){
            oss << data_json_["imu"].at("acc")[i] << ",";
        }
        for(int i=0;i<data_json_["imu"].at("omg").size();++i){
            oss << data_json_["imu"].at("omg")[i] << ",";
        }
        oss << data_json_["state"].at("current_state") << ",";
        for(int i=0;i<scope_data_.size();++i){
            oss << scope_data_[i] << ",";
        }
        oss << "\n";
        file_ << oss.str();
    }
};

#endif
