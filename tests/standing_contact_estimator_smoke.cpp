#include <cmath>
#include <iostream>

#include "ContactPhase.hpp"

int main()
{
    for (int sample = 0; sample <= 100; ++sample) {
        const float gait_phase = static_cast<float>(sample) / 100.0f;
        const float standing_phase = EstimatorContactPhase(true, gait_phase);
        const float trust = LinearKfContactTrust(standing_phase);
        if (std::abs(standing_phase - 0.5f) > 1e-7f ||
            std::abs(trust - 1.0f) > 1e-7f) {
            std::cerr << "Standing estimator phase/trust changed at gait phase "
                      << gait_phase << std::endl;
            return 1;
        }
        if (std::abs(EstimatorContactPhase(false, gait_phase) - gait_phase) > 1e-7f) {
            std::cerr << "Locomotion estimator phase was modified" << std::endl;
            return 2;
        }
    }

    std::cout << "[TEST] standing_contact_estimator_smoke PASSED!" << std::endl;
    return 0;
}
