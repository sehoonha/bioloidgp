/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef ROBOT_HUMANOIDCONTROLLER_H
#define ROBOT_HUMANOIDCONTROLLER_H

#include <Eigen/Dense>
#include "utils/HppCommon.h"

namespace dart {
namespace dynamics {
class BodyNode;
class Skeleton;
}  // namespace dynamics
namespace constraint {
class ConstraintSolver;
}  // namespace constraint
}  // namespace dart

namespace bioloidgp {
namespace robot {
class MotorMap;
} // namespace robot
} // namespace bioloidgp


namespace bioloidgp {
namespace robot {

class HumanoidController {
public:
    HumanoidController(
        dart::dynamics::Skeleton* _robot,
        dart::constraint::ConstraintSolver* _collisionSolver
        );
    virtual ~HumanoidController();

    virtual void update(double _currentTime);
    virtual void printDebugInfo() const;
    void keyboard(unsigned char _key, int _x, int _y, double _currentTime);

protected:
    void setJointDamping(double _damping = 80.0);

protected:
    MEMBER_PTR(dart::dynamics::Skeleton*, robot);
    MEMBER_PTR(dart::constraint::ConstraintSolver*, collisionSolver);
    MEMBER_PTR(MotorMap*, motormap);

    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;

}; // class Humanoidcontroller

} // namespace robot
} // namespace bioloidgp



#endif // #ifndef ROBOT_HUMANOIDCONTROLLER_H

