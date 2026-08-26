#pragma once

#include <memory>
#include "WbcType.h"

/**
 * CMPCBridge provides a clean, thread-safe, non-singleton wrapper around GaitCtrller.
 * It encapsulates GaitCtrller without leaking internal CMPC or Pinocchio headers into FSM states.
 */
class CMPCBridge {
public:
    explicit CMPCBridge(double freq, double* pidParam4, bool wbic_enabled = true);
    ~CMPCBridge();

    CMPCBridge(const CMPCBridge&) = delete;
    CMPCBridge& operator=(const CMPCBridge&) = delete;
    CMPCBridge(CMPCBridge&&) noexcept;
    CMPCBridge& operator=(CMPCBridge&&) noexcept;

    void SetGaitType(int gaitType);
    void SetRobotMode(int mode);
    void SetRobotVel(double* vel3);

    void SetWbicEnabled(bool enabled);
    bool IsWbicEnabled() const;
    void Reset();

    void TorqueCalculator(double* imuData10,
                          double* motorData24,
                          double* effort12,
                          wbic::JointHybridCommand* hybridCmd = nullptr);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
