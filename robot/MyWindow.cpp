/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot/MyWindow.h"

#include "dart/math/Helpers.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/gui/GLFuncs.h"

#include "utils/GLObjects.h"
#include "utils/CppCommon.h"
#include "robot/HumanoidController.h"


//==============================================================================
MyWindow::MyWindow(bioloidgp::robot::HumanoidController* _controller)
    : SimWindow(),
      mController(_controller)
{
    mForce = Eigen::Vector3d::Zero();
    mImpulseDuration = 0.0;

    // 0.166622 0.548365 0.118241 0.810896
    Eigen::Quaterniond q(0.810896, 0.166622, 0.548365, 0.118241);
    mTrackBall.setQuaternion(q);
    mTrans = Eigen::Vector3d(-32.242,  212.85, 21.7107);
}

//==============================================================================
MyWindow::~MyWindow()
{
    delete mController;
}

int g_cnt = 0;
//==============================================================================
void MyWindow::timeStepping()
{
    // External force
    mWorld->getSkeleton(0)->getBodyNode("torso")->addExtForce(
          mForce);

    // Internal force
    mController->update(mWorld->getTime());

    // simulate one step
    mWorld->step();
    
    if (g_cnt % 500 == 0) {
        calculateInertia();
    }
    g_cnt++;
    
    // for perturbation test
    mImpulseDuration--;
    if (mImpulseDuration <= 0)
    {
        mImpulseDuration = 0;
        mForce.setZero();
    }
}

//==============================================================================
void MyWindow::drawSkels()
{
    // Eigen::Quaterniond q = mTrackBall.getCurrQuat();
    // cout << "Quat = " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    // cout << mTrans.transpose() << endl;
    // cout << mWorld->getSkeleton(0)->getPositions().transpose() << endl;

//  glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
        if (i == 1) {
            glPushMatrix();
            glTranslated(0, -0.301, 0);
            bioloidgp::utils::renderChessBoard(100, 100, 50.0, 50.0);
            glPopMatrix();

            glPushMatrix();
            glTranslated(0, -0.001, 0);
            mWorld->getSkeleton(i)->draw(mRI);
            glPopMatrix();
            
            continue;
        }
        mWorld->getSkeleton(i)->draw(mRI);
        // {
        //     Eigen::Vector3d C = mWorld->getSkeleton(i)->getWorldCOM();
        //     glPushMatrix();
        //     glTranslated(C(0), C(1), C(2));
        //     bioloidgp::utils::renderAxis(1.0);
        //     glPopMatrix();
        // }
        
    }

    // draw arrow
    if (mImpulseDuration > 0)
    {
        Eigen::Vector3d poa
            =  mWorld->getSkeleton(0)->getBodyNode(
                "torso")->getTransform()
            * Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d start = poa - mForce / 500.0;
        double len = mForce.norm() / 500.0;
        dart::gui::drawArrow3D(start, mForce, len, 0.05, 0.1);
    }
}

void MyWindow::calculateInertia() {
    dart::dynamics::Skeleton* robot = mWorld->getSkeleton(0);
    int n = robot->getNumBodyNodes();

    double I = 0.0;
    double m = 0;
    double OFFSET = -0.30;
    for (int i = 0; i < n; i++) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        m += bn->getMass();
        Eigen::Vector3d p = bn->getWorldCOM();
        double y = p.y() - (OFFSET); // Subtract the feet height
        double z = p.z();
        // cout << bn->getName() << " " << y << " " << z << endl;
        I += bn->getMass() * (y * y + z * z);
    }
    Eigen::Vector3d C = robot->getWorldCOM();
    cout << m << ", " << I << ", " << C.y() - OFFSET << ", " << C.z() << endl;
    
}


//==============================================================================
int temp = 0;
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
    switch (_key)
    {
    case ' ':  // use space key to play or stop the motion
        mSimulating = !mSimulating;
        if (mSimulating)
        {
            mPlay = false;
            glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case 'p':  // playBack
        mPlay = !mPlay;
        if (mPlay)
        {
            mSimulating = false;
            glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[':  // step backward
        if (!mSimulating)
        {
            mPlayFrame-=10;
            if (mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']':  // step forwardward
        if (!mSimulating)
        {
            mPlayFrame+=10;
            if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case 'I':  // print debug information
        calculateInertia();
        break;
    case 'i':  // print debug information
        mController->printDebugInfo();
        break;
    case 'v':  // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
    case 'a':  // upper right force
        mForce[0] = 5;
        mImpulseDuration = 100;
        std::cout << "push forward" << std::endl;
        break;
    case 's':  // upper right force
        mForce[0] = -5;
        mImpulseDuration = 100;
        std::cout << "push backward" << std::endl;
        break;
    case 'd':  // upper right force
        mForce[2] = 5;
        mImpulseDuration = 100;
        std::cout << "push right" << std::endl;
        break;
    case 'f':  // upper right force
        mForce[2] = -5;
        mImpulseDuration = 100;
        std::cout << "push left" << std::endl;
        break;
    case 'n':  // upper right force
        mController->setMotionTargetPose(temp);
        temp++;
    default:
        Win3D::keyboard(_key, _x, _y);
    }

    // Keyboard control for Controller
    mController->keyboard(_key, _x, _y, mWorld->getTime());

    glutPostRedisplay();
}

