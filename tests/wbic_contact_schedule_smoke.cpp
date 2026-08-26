#include <iostream>

#include "ContactSchedule.hpp"

int main()
{
    if (PlannedContactFromSwingState(0.25f)) {
        std::cerr << "Positive swing state was classified as stance" << std::endl;
        return 1;
    }
    if (!PlannedContactFromSwingState(0.0f)) {
        std::cerr << "Zero swing state was classified as swing" << std::endl;
        return 2;
    }

    const float continuous_stance_phase = 0.25f;
    const bool planned_contact = PlannedContactFromSwingState(0.0f);
    if (continuous_stance_phase >= 0.5f || !planned_contact) {
        std::cerr << "Discrete contact was incorrectly derived from continuous phase"
                  << std::endl;
        return 3;
    }

    std::cout << "[TEST] wbic_contact_schedule_smoke PASSED!" << std::endl;
    return 0;
}
