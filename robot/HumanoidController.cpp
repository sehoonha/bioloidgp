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
#include "MotorMap.h"
#include "Motion.h"

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
    const int NDOFS = robot()->getDof();
    const int NMOTORS = 18;

    setJointDamping();

    set_motormap( new MotorMap(NMOTORS, NDOFS) );
    motormap()->load("data/urdf/BioloidGP/BioloidGPMotorMap.xml");

    set_motion( new Motion(NMOTORS) );
    motion()->load("data/xml/motion.xml");

    mKp = Eigen::VectorXd::Zero(NDOFS);
    mKd = Eigen::VectorXd::Zero(NDOFS);
    for (int i = 6; i < NDOFS; ++i) {
        mKp(i) = 600.0;
        mKd(i) = 1.0;
    }
    for (int i = 0; i < robot()->getNumBodyNodes(); i++) {
        cout << "Joint " << i + 5 << " : " << robot()->getJoint(i)->getName() << endl;
    }
}

HumanoidController::~HumanoidController() {
}

void HumanoidController::update(double _currentTime) {
    const int NDOFS = robot()->getDof();
    Eigen::VectorXd q    = robot()->getPositions();
    Eigen::VectorXd dq   = robot()->getVelocities();
    // Eigen::VectorXd qhat = Eigen::VectorXd::Zero(NDOFS);
    Eigen::VectorXd tau  = Eigen::VectorXd::Zero(NDOFS);

    // Eigen::VectorXd motor_qhat(18);
    // motor_qhat <<
    //     1.0, 1.0, -0.5, 0.5, 0.0, 0.0,
    //     0.0, 0.0,
    //     0.0, 0.0,  0.6, 0.6,  -1.0, -1.0,  0.5, 0.5,  0.0, 0.0;

    Eigen::VectorXd motor_qhat = motion()->targetPose(_currentTime);
    Eigen::VectorXd qhat = motormap()->fromMotorMapVector( motor_qhat );


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







