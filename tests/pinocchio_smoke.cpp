#include <iostream>

#include <Eigen/Core>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/spatial/inertia.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

int main()
{
    pinocchio::Model model;

    const auto root = model.addJoint(
        0,
        pinocchio::JointModelFreeFlyer(),
        pinocchio::SE3::Identity(),
        "root_joint");

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    model.appendBodyToJoint(
        root,
        pinocchio::Inertia(
            10.0,
            Eigen::Vector3d::Zero(),
            I),
        pinocchio::SE3::Identity());

    pinocchio::Data data(model);

    Eigen::VectorXd q = pinocchio::neutral(model);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    pinocchio::crba(model, data, q);

    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose()
              .triangularView<Eigen::StrictlyLower>();

    pinocchio::computeCoriolisMatrix(
        model, data, q, v);

    pinocchio::computeGeneralizedGravity(
        model, data, q);

    pinocchio::nonLinearEffects(
        model, data, q, v);

    std::cout << "nq=" << model.nq
              << " nv=" << model.nv << "\n";

    std::cout
        << "M symmetry error = "
        << (data.M - data.M.transpose()).norm()
        << "\n";

    std::cout
        << "Cv consistency error = "
        << (data.C * v - (data.nle - data.g)).norm()
        << "\n";

    if (model.nq != 7 || model.nv != 6)
        return 1;

    if (!data.M.allFinite())
        return 2;

    std::cout << "PINOCCHIO HEADER-ONLY OK\n";

    return 0;
}
