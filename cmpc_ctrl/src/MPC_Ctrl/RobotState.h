#ifndef _RobotState
#define _RobotState

#include <Eigen/Dense>
#include "Utilities/common_types.h"
#include "RobotConfig.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w; // Position, velocity, angular velocity
        Matrix<fpt,3,4> r_feet; // Foot positions in body frame
        Matrix<fpt,3,3> R;       // Rotation matrix from body to world
        Matrix<fpt,3,3> R_yaw;   // Yaw rotation matrix
        Matrix<fpt,3,3> I_body;  // Body inertia matrix
        Quaternionf q;           // Orientation quaternion
        fpt yaw;                 // Yaw angle
        fpt m = RobotConfig::MASS; // Robot mass (configured in RobotConfig.h)
    //private:
};
#endif
