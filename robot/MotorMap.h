/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef ROBOT_MOTORMAP_H
#define ROBOT_MOTORMAP_H

#include <vector>
#include <Eigen/Dense>
#include "utils/HppCommon.h"

namespace bioloidgp {
namespace robot {

struct Motor;

class MotorMap {
public:
    MotorMap(int _maxMotorID, int _numFullDimensions);
    virtual ~MotorMap();

    bool load(const char* const filename);

    void setNumMotors(int _n) { mMaxMotorIDs = _n; }
    int numMotors() const { return mMaxMotorIDs; }
    void setNumFullDimensions(int _n) { mNumFullDimensions =_n; }
    int numFullDimensions() const { return mNumFullDimensions; }
    Motor* motor(int index) { return motors[index]; }

    int findMotorIndex(int motorID) const;
    int findMotorIndex(const char* const motorName) const;

    Eigen::VectorXd toMotorMapVector(const Eigen::VectorXd& v) const;
    Eigen::VectorXd fromMotorMapVector(const Eigen::VectorXd& mtv) const;

protected:
    std::vector<Motor*> motors;
    int mMaxMotorIDs;
    int mNumFullDimensions;
}; // class MotorMap

} // namespace robot
} // namespace bioloidgp

#endif // #ifndef ROBOT_MOTORMAP_H

