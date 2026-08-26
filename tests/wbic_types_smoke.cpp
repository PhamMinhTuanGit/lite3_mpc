#include <limits>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include "Lite3DynamicModel.hpp"
#include "WbcType.h"

int main()
{
    Lite3Dynamics dynamics;
    const auto& model = dynamics.model();

    wbic::FootFrameIds foot_ids{{
        dynamics.footFrameId(Lite3Dynamics::Leg::FR),
        dynamics.footFrameId(Lite3Dynamics::Leg::FL),
        dynamics.footFrameId(Lite3Dynamics::Leg::HR),
        dynamics.footFrameId(Lite3Dynamics::Leg::HL),
    }};
    const wbic::Configuration q = pinocchio::neutral(model);
    const wbic::GeneralizedVelocity v =
        wbic::GeneralizedVelocity::Zero();
    wbic::DynamicsInput dynamics_input(model, q, v, foot_ids);

    if (dynamics_input.validate() != wbic::InputStatus::Ok) {
        return 1;
    }

    dynamics_input.q[6] = 2.0;
    if (dynamics_input.validate()
        != wbic::InputStatus::InvalidBaseQuaternion) {
        return 2;
    }
    dynamics_input.q = q;
    dynamics_input.foot_ids[1] = dynamics_input.foot_ids[0];
    if (dynamics_input.validate() != wbic::InputStatus::DuplicateFootFrame) {
        return 3;
    }

    wbic::ContactAssemblyInput contact_input;
    if (contact_input.validate() != wbic::InputStatus::Ok) {
        return 4;
    }
    for (std::size_t leg = 0; leg < wbic::kNumLegs; ++leg) {
        if (!contact_input.f_mpc[leg].isZero()
            || !contact_input.Jf[leg].isZero()
            || !contact_input.dJdq_f[leg].isZero()) {
            return 5;
        }
    }

    contact_input.phase[0] = 1.01;
    if (contact_input.validate() != wbic::InputStatus::InvalidContactPhase) {
        return 6;
    }
    contact_input.phase[0] = 0.5;
    contact_input.f_mpc[2][1] = std::numeric_limits<double>::quiet_NaN();
    if (contact_input.validate() != wbic::InputStatus::NonFiniteValue) {
        return 7;
    }

    return 0;
}
