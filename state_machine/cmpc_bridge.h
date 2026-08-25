#pragma once

#include "MPC_Ctrl/CmpcTelemetry.h"
#include "Controllers/GeneralizedMomentumObserver.h"

// GaitCtrller.h is NOT included here.
// It defines extern "C" wrappers inline, so including it in multiple TUs
// causes multiple-definition link errors. CMPCBridge calls those functions
// via forward-declared extern "C" linkage (resolved from GaitCtrller.cpp).
class CMPCBridge {
public:
    CMPCBridge(
        double freq,
        double* pidParam4,
        const GMOConfig& gmoConfig = GMOConfig{});
    ~CMPCBridge();

    void SetGaitType(int gaitType);
    void SetRobotMode(int mode);
    void SetRobotVel(double* vel3);
    void SetObservationTimestamp(double timestampSeconds);
    void TorqueCalculator(double* imuData10, double* motorData24, double* effort12, CmpcTelemetryData* telem = nullptr);
};
