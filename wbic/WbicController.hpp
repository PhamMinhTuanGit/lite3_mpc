#pragma once

#include <memory>
#include "WbcType.h"

namespace wbic {

class WbicController {
public:
    explicit WbicController(const WbicConfig& config = WbicConfig());
    ~WbicController();

    WbicController(const WbicController&) = delete;
    WbicController& operator=(const WbicController&) = delete;
    WbicController(WbicController&&) noexcept;
    WbicController& operator=(WbicController&&) noexcept;

    void Reset() noexcept;
    WbicStatus Run(const WbicInput& input, WbicOutput* output) noexcept;

    const WbicConfig& GetConfig() const noexcept;
    void SetConfig(const WbicConfig& config) noexcept;

    bool IsLatched() const noexcept;
    void Unlatch() noexcept;
    int GetConsecutiveFailures() const noexcept;

    /**
     * Helper to populate JointHybridCommand POD from WbicOutput.
     */
    static void PopulateHybridCommand(const WbicOutput& output,
                                      JointHybridCommand* cmd) noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace wbic
