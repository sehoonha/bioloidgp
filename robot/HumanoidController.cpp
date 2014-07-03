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

    setJointDamping(0.15);

    set_motormap( new MotorMap(NMOTORS, NDOFS) );
    motormap()->load(DATA_DIR"/urdf/BioloidGP/BioloidGPMotorMap.xml");

    set_motion( new Motion(NMOTORS) );
    // motion()->load(DATA_DIR"/xml/motion.xml");
    // motion()->loadMTN(DATA_DIR"/mtn/bio_gp_humanoid_kr.mtn", "HandStanding");
    motion()->loadMTN(DATA_DIR"/mtn/bio_gp_humanoid_kr.mtn", "SideDumbling");
    motion()->printSteps();
    // exit(0);

    mKp = Eigen::VectorXd::Zero(NDOFS);
    mKd = Eigen::VectorXd::Zero(NDOFS);
    for (int i = 6; i < NDOFS; ++i) {
        mKp(i) = 600.0;
        mKd(i) = 1.0;
    }
    for (int i = 0; i < robot()->getNumBodyNodes(); i++) {
        LOG(INFO) << "Joint " << i + 5 << " : name = " << robot()->getJoint(i)->getName();
    }
}

HumanoidController::~HumanoidController() {
}

void HumanoidController::setMotorMapPose(const Eigen::VectorXd& mtv) {
    int n = robot()->getDof();
    Eigen::VectorXd q = robot()->getPositions();
    Eigen::VectorXd pose = motormap()->fromMotorMapVector( mtv );
    CHECK_EQ(q.size(), pose.size());
    q.tail(n - 6) = pose.tail(n - 6);
    LOG(INFO) << "q = " << q.transpose();

    robot()->setPositions(q);
    robot()->computeForwardKinematics(true, true, false);
    
}

void HumanoidController::setMotionTargetPose(int index) {
    Eigen::VectorXd mtv = motion()->targetPoseAtIndex(index);
    LOG(INFO) << FUNCTION_NAME();
    LOG(INFO) << index << " : " << mtv.transpose();
    this->setMotorMapPose(mtv);
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


    // int m = motormap()->numMotors();
    // Eigen::VectorXd mtvInitPose = Eigen::VectorXd::Ones(m) * 512;
    // mtvInitPose <<
    //     342, 681, 572, 451, 762, 261,
    //     358, 666,
    //     515, 508, 741, 282, 857, 166, 684, 339, 515, 508;
    // Eigen::VectorXd qhat = motormap()->fromMotorMapVector( mtvInitPose );

    Eigen::VectorXd motor_qhat = motion()->targetPose(_currentTime);
    Eigen::VectorXd qhat = motormap()->fromMotorMapVector( motor_qhat );


    tau.head<6>() = Eigen::Vector6d::Zero();
    for (int i = 6; i < NDOFS; ++i) {
        tau(i) = -mKp(i) * (q(i) - qhat(i))
            -mKd(i) * dq(i);
    }


    // Confine within the limit: 25% of stall torque
    // Reference: http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm
    const double MAX_TORQUE = 0.5 * 1.5;
    for (int i = 6; i < NDOFS; i++) {
        tau(i) = CONFINE(tau(i), -MAX_TORQUE, MAX_TORQUE);
    }
    // cout << _currentTime << " : " << tau.transpose() << endl;
    
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







