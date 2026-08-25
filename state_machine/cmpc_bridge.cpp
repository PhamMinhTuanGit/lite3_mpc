// Compiled in isolation to avoid name conflicts with the main project.
// GaitCtrller.h defines extern "C" functions inline, so it must be included in
// exactly one TU (GaitCtrller.cpp already does).  Here we only forward-declare
// those functions to resolve them via normal linking.
#include "cmpc_bridge.h"
#include <cstring>

struct JointEff { double eff[12]; };

extern "C" {
    void init_controller(double freq, double pidParam[]);
    void init_controller_with_gmo(
        double freq, double pidParam[], int enabled, double gain,
        double forceDamping);
    void set_gait_type(int gaitType);
    void set_robot_mode(int mode);
    void set_robot_vel(double vel[]);
    JointEff* torque_calculator(double imuData[], double motorData[]);
    JointEff* torque_calculator_with_telem(double imuData[], double motorData[], CmpcTelemetryData* telem);
}

CMPCBridge::CMPCBridge(
    double freq,
    double* pidParam4,
    const GMOConfig& gmoConfig) {
    init_controller_with_gmo(
        freq, pidParam4, gmoConfig.enabled ? 1 : 0,
        gmoConfig.gain, gmoConfig.force_damping);
}

CMPCBridge::~CMPCBridge() = default;

void CMPCBridge::SetGaitType(int g) { set_gait_type(g); }
void CMPCBridge::SetRobotMode(int m) { set_robot_mode(m); }
void CMPCBridge::SetRobotVel(double* v) { set_robot_vel(v); }

void CMPCBridge::TorqueCalculator(double* imu, double* motor, double* effort, CmpcTelemetryData* telem) {
    JointEff* res = nullptr;
    if (telem != nullptr) {
        res = torque_calculator_with_telem(imu, motor, telem);
    } else {
        res = torque_calculator(imu, motor);
    }
    std::memcpy(effort, res->eff, 12 * sizeof(double));
}
