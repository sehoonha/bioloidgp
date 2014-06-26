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

#include <iostream>

#include "utils/CppCommon.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/sdf/SoftSdfParser.h"
#include "dart/utils/urdf/DartLoader.h"

#include "robot/MyWindow.h"
#include "robot/HumanoidController.h"
// #include "robot/Controller.h"

using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

int main(int argc, char* argv[])
{
    srand( (unsigned int) time (NULL) );
    // Create empty soft world
    World* myWorld = new World;

    // Load ground and Atlas robot and add them to the world
    DartLoader urdfLoader;
    Skeleton* ground = urdfLoader.parseSkeleton(
        DATA_DIR"/sdf/ground.urdf");
    Skeleton* robot
        = urdfLoader.parseSkeleton(
            DATA_DIR"/urdf/BioloidGP/BioloidGP.URDF");
    robot->enableSelfCollision();

    myWorld->addSkeleton(robot);
    myWorld->addSkeleton(ground);

    // Set initial configuration for Atlas robot
    Eigen::VectorXd q = robot->getPositions();
    q[0] = -0.5 * DART_PI;
    Eigen::VectorXd noise = 0.02 * Eigen::VectorXd::Random( q.size() );
    robot->setPositions(q + noise);
    robot->computeForwardKinematics(true, true, false);

    // Set gravity of the world
    myWorld->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

    // Create a humanoid controller
    bioloidgp::robot::HumanoidController* con =
        new bioloidgp::robot::HumanoidController(robot, myWorld->getConstraintSolver());

    // Create a window and link it to the world
    // MyWindow window(new Controller(robot, myWorld->getConstraintSolver()));
    MyWindow window(con);
    window.setWorld(myWorld);

    // Print manual
    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << endl;

    // Run glut loop
    glutInit(&argc, argv);
    window.initWindow(1280, 720, "BioloidGP Robot - with Dart4.0");

    glutMainLoop();

    return 0;
}
