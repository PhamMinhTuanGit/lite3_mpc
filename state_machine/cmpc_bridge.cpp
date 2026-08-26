#include "cmpc_bridge.h"
#include "GaitCtrller.h"

class CMPCBridge::Impl {
public:
    explicit Impl(double freq, double* pidParam4, bool wbic_enabled)
        : controller_(std::make_unique<GaitCtrller>(freq, pidParam4, wbic_enabled))
    {
    }

    void SetGaitType(int gaitType)
    {
        if (controller_) controller_->SetGaitType(gaitType);
    }

    void SetRobotMode(int mode)
    {
        if (controller_) controller_->SetRobotMode(mode);
    }

    void SetRobotVel(double* vel3)
    {
        if (controller_) controller_->SetRobotVel(vel3);
    }

    void SetWbicEnabled(bool enabled)
    {
        if (controller_) controller_->SetWbicEnabled(enabled);
    }

    bool IsWbicEnabled() const
    {
        return controller_ ? controller_->IsWbicEnabled() : false;
    }

    void Reset()
    {
        if (controller_) controller_->Reset();
    }

    void TorqueCalculator(double* imuData10,
                          double* motorData24,
                          double* effort12,
                          wbic::JointHybridCommand* hybridCmd)
    {
        if (controller_) {
            controller_->TorqueCalculator(imuData10, motorData24, effort12, hybridCmd);
        }
    }

private:
    std::unique_ptr<GaitCtrller> controller_;
};

CMPCBridge::CMPCBridge(double freq, double* pidParam4, bool wbic_enabled)
    : impl_(std::make_unique<Impl>(freq, pidParam4, wbic_enabled))
{
}

CMPCBridge::~CMPCBridge() = default;

CMPCBridge::CMPCBridge(CMPCBridge&&) noexcept = default;
CMPCBridge& CMPCBridge::operator=(CMPCBridge&&) noexcept = default;

void CMPCBridge::SetGaitType(int gaitType)
{
    if (impl_) impl_->SetGaitType(gaitType);
}

void CMPCBridge::SetRobotMode(int mode)
{
    if (impl_) impl_->SetRobotMode(mode);
}

void CMPCBridge::SetRobotVel(double* vel3)
{
    if (impl_) impl_->SetRobotVel(vel3);
}

void CMPCBridge::SetWbicEnabled(bool enabled)
{
    if (impl_) impl_->SetWbicEnabled(enabled);
}

bool CMPCBridge::IsWbicEnabled() const
{
    return impl_ ? impl_->IsWbicEnabled() : false;
}

void CMPCBridge::Reset()
{
    if (impl_) impl_->Reset();
}

void CMPCBridge::TorqueCalculator(double* imuData10,
                                  double* motorData24,
                                  double* effort12,
                                  wbic::JointHybridCommand* hybridCmd)
{
    if (impl_) impl_->TorqueCalculator(imuData10, motorData24, effort12, hybridCmd);
}
