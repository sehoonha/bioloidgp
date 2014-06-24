/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "HumanoidController.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "utils/CppCommon.h"

namespace bioloidgp {
namespace robot {
////////////////////////////////////////////////////////////
// class HumanoidController implementation
HumanoidController::HumanoidController(
    dart::dynamics::Skeleton* _robot,
    dart::constraint::ConstraintSolver* _collisionSolver
    )
    : MEMBER_INIT(robot, _robot)
    , MEMBER_INIT(collisionSolver, _collisionSolver)

{
    setJointDamping();

    const int NDOFS = robot()->getDof();
    mKp = Eigen::VectorXd::Zero(NDOFS);
    mKd = Eigen::VectorXd::Zero(NDOFS);
    for (int i = 6; i < NDOFS; ++i) {
        mKp(i) = 1000.0;
        mKd(i) = 1.0;
    }
}

HumanoidController::~HumanoidController() {
}

void HumanoidController::update(double _currentTime) {
    const int NDOFS = robot()->getDof();
    Eigen::VectorXd q    = robot()->getPositions();
    Eigen::VectorXd dq   = robot()->getVelocities();
    Eigen::VectorXd qhat = Eigen::VectorXd::Zero(NDOFS);
    Eigen::VectorXd tau  = Eigen::VectorXd::Zero(NDOFS);

    tau.head<6>() = Eigen::Vector6d::Zero();
    for (int i = 6; i < NDOFS; ++i) {
        tau(i) = -mKp(i) * (q(i) - qhat(i))
            -mKd(i) * dq(i);
    }
    
    robot()->setForces(tau);

}

void HumanoidController::setJointDamping(double _damping) {
    for (int i = 1; i < robot()->getNumBodyNodes(); ++i) {
        dart::dynamics::Joint* joint = robot()->getJoint(i);
        if (joint->getDof() > 0) {
            for (int j = 0; j < joint->getDof(); ++j) {
                joint->setDampingCoefficient(j, _damping);
            }
        }
    }
}

void HumanoidController::printDebugInfo() const {
}

void HumanoidController::keyboard(unsigned char _key, int _x, int _y, double _currentTime) {
}

// class HumanoidController ends
////////////////////////////////////////////////////////////


} // namespace robot
} // namespace bioloidgp







