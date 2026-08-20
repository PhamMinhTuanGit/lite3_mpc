// Compiled in isolation to avoid name conflicts with the main project.
// GaitCtrller.h defines extern "C" functions inline, so it must be included in
// exactly one TU (GaitCtrller.cpp already does).  Here we only forward-declare
// those functions to resolve them via normal linking.
#include "cmpc_bridge.h"
#include <cstring>

struct JointEff { double eff[12]; };

extern "C" {
    void init_controller(double freq, double pidParam[]);
    void set_gait_type(int gaitType);
    void set_robot_mode(int mode);
    void set_robot_vel(double vel[]);
    JointEff* torque_calculator(double imuData[], double motorData[]);
}

CMPCBridge::CMPCBridge(double freq, double* pidParam4) {
    init_controller(freq, pidParam4);
}

CMPCBridge::~CMPCBridge() = default;

void CMPCBridge::SetGaitType(int g) { set_gait_type(g); }
void CMPCBridge::SetRobotMode(int m) { set_robot_mode(m); }
void CMPCBridge::SetRobotVel(double* v) { set_robot_vel(v); }

void CMPCBridge::TorqueCalculator(double* imu, double* motor, double* effort) {
    JointEff* res = torque_calculator(imu, motor);
    std::memcpy(effort, res->eff, 12 * sizeof(double));
}
