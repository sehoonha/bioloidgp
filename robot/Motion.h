/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include <vector>
#include <Eigen/Dense>
#include "utils/HppCommon.h"

namespace bioloidgp {
namespace robot {

struct Step {
    Step();
    double pause;
    double duration;
    Eigen::VectorXd targetpose;
}; // struct Motion

struct Motion {
    Motion(int _dim, const Eigen::VectorXd& _init);
    bool load(const char* const filename);
    bool loadMTN(const char* const filename, const char* const motionname);

    void setInitialPose(const Eigen::VectorXd& _init) { initialPose = _init; }

    Eigen::VectorXd targetPoseAtIndex(int i) const;
    Eigen::VectorXd targetPose(double t) const;

    void printSteps();

    int dim;
    int stepIndex;
    std::vector<Step> steps;
    Eigen::VectorXd initialPose;
}; // struct Motion

} // namespace robot
} // namespace bioloidgp



#endif // #ifndef ROBOT_MOTION_H

