/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "Motion.h"
// Standard Libraries
#include <sstream>
#include <fstream>
// Boost Libraries
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
// External Libraries
#include <tinyxml2.h>
// Local headers
#include "utils/CppCommon.h"

namespace bioloidgp {
namespace robot {

////////////////////////////////////////////////////////////
// struct Step implementation
Step::Step()
    : duration(-1.0)
{
}

void parseStep(Step& s, int dim, tinyxml2::XMLElement* xnStep) {
    s.duration = xnStep->DoubleAttribute("duration");
    std::stringstream sin;
    sin.str( xnStep->GetText() );
    s.targetpose = Eigen::VectorXd::Zero(dim);
    for (int i = 0; i < s.targetpose.size(); i++) {
        sin >> s.targetpose(i);
    }
    // LOG(INFO) << "step. duration = " << s.duration << " : pose = "
    //           << s.targetpose.transpose()
    //           << " (" << s.targetpose.size() << ")";
}

// struct Step ends
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// struct Motion implementation
Motion::Motion(int _dim, const Eigen::VectorXd& _init)
    : dim(_dim)
    , stepIndex(-1)
{
    setInitialPose(_init);
}

bool Motion::load(const char* const filename) {
    LOG(INFO) << FUNCTION_NAME() << " [" << filename << "]";
    // Read XML
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result
        = doc.LoadFile(filename);
    if (result != tinyxml2::XML_NO_ERROR) {
        doc.PrintError();
        LOG(ERROR) << "ErrorStr1: " << doc.GetErrorStr1();
        LOG(ERROR) << "ErrorStr2: " << doc.GetErrorStr2();
        return false;
    }
    LOG(INFO) << "read XML OK";

    steps.clear();
    tinyxml2::XMLElement* xnMotion = doc.FirstChildElement("motion");
    for (tinyxml2::XMLElement* xnStep = xnMotion->FirstChildElement("step");
         xnStep; xnStep = xnStep->NextSiblingElement("step")) {
        Step s;
        parseStep(s, dim, xnStep);
        steps.push_back(s);
    }
    LOG(INFO) << FUNCTION_NAME() << " OK"; 
    return true;
}

bool Motion::loadMTN(const char* const filename, const char* const motionname) {
    using boost::lexical_cast;
    
    LOG(INFO) << FUNCTION_NAME() << " file : [" << filename << "]";
    LOG(INFO) << "motionname =  [" << motionname << "]";

    std::ifstream fin(filename);
    if (fin.is_open() == false) {
        LOG(ERROR) << "cannot open the file.";
        return false;
    }

    int pageIndex = 1;
    std::string pageName = "";

    int pageNextPageIndex = -1;
    int pageExitPageIndex = 0;
    int pageRepeatTime = 1;
    double pageSpeedRate = 1.0;
    int pageCtrlInertialForce = 1;

    Eigen::VectorXd stepPose;
    double stepPause;
    double stepTime;

    bool acceptPage = false;

    double angleAtMin = -(5.0 / 6.0) * UTILS_PI;
    double angleAtMax =  (5.0 / 6.0) * UTILS_PI;

    while(true) {
        const int MAX_LINE = 512;
        char buf[MAX_LINE + 2];

        fin.getline(buf, MAX_LINE);
        if (fin.fail()) {
            break;
        }
        // cout << "line = [" << buf << "]" << endl;

        // For each line
        std::vector<std::string> tokens;
        boost::split(tokens, buf, boost::is_any_of("= "));

        // If the line is empty
        if (tokens.size() == 0) {
            continue;
        }

        // Only accept the alphabet as command
        std::string cmd = "";
        for (int i = 0; i < tokens[0].size(); i++) {
            char c = tokens[0][i];
            if (std::isalpha(c) || std::isdigit(c) || c == '_') {
                cmd += c;
            }
        }

        if (cmd == "type") {
            // Do nothing
        } else if (cmd == "version") {
            // Do nothing
        } else if (cmd == "enable") {
            // Do nothing
        } else if (cmd == "motor_type") {
            // Do nothing
        } else if (cmd == "page_begin") {
        } else if (cmd == "page_end") {
            acceptPage = false;
            pageIndex++;
        } else if (cmd == "name") {
            pageName = tokens[1];
            if (pageName == motionname || pageIndex == pageNextPageIndex) {
                acceptPage = true;
            }
            if (acceptPage) {
                LOG(INFO) << "command = " << cmd << " : pageName = {" << pageName << "} at " << pageIndex;
            }
        } else if (cmd == "compliance") {
        } else if (cmd == "play_param") {
            if (acceptPage) {
                pageNextPageIndex     = lexical_cast<int>(tokens[1]);
                pageExitPageIndex     = lexical_cast<int>(tokens[2]);
                pageRepeatTime        = lexical_cast<int>(tokens[3]);
                pageSpeedRate         = lexical_cast<double>(tokens[4]);
                pageCtrlInertialForce = lexical_cast<int>(tokens[5]);
            
                LOG(INFO) << "command = " << cmd << ". "
                          << "pageNextPageIndex = " << pageNextPageIndex << " "
                          << "pageExitPageIndex = " << pageExitPageIndex << " "
                          << "pageRepeatTime = " << pageRepeatTime << " "
                          << "pageSpeedRate = " << pageSpeedRate << " "
                          << "pageCtrlInertialForce = " << pageCtrlInertialForce << " ";
            }
        } else if (cmd == "step") {
            if (acceptPage) {
                int n = tokens.size();

                stepPose = Eigen::VectorXd::Zero(dim);
                for (int i = 0; i < stepPose.size(); i++) {
                    // The first one is command, the second one is "zero" indexed motor
                    double v = lexical_cast<double>(tokens[i + 2]); 
                    stepPose(i) = v;
                }
                // Remove the first entry
                // stepPose = stepPose.segment(1, dim);
                
                // Read the rest of parameters
                stepPause = lexical_cast<double>(tokens[n - 2]);
                stepTime  = lexical_cast<double>(tokens[n - 1]);
                // LOG(INFO) << "command = " << cmd << ". "
                //           << "stepPause = " << stepPause << " "
                //           << "stepTime = " << stepTime << " "
                //           << "stepPose = " << stepPose.transpose();
                Step s;
                // s.duration = (stepPause + stepTime) / pageSpeedRate;
                s.duration = stepTime / pageSpeedRate;
                s.pause    = stepPause / pageSpeedRate;
                s.targetpose = stepPose;
                steps.push_back(s);
            }
        } else if (cmd == "1c") {
            // Do nothing
        } else {
            LOG(FATAL) << "unknown command: [" << cmd << "]";
        }
    }
    
    return true;
}

Eigen::VectorXd Motion::targetPoseAtIndex(int i) const {
    i = (i % steps.size());
    const Step& s = steps[i];
    return s.targetpose;
}

Eigen::VectorXd Motion::targetPose(double t) const {
    // If there are no steps..
    if (steps.size() == 0) {
        return initialPose;
    }

    Eigen::VectorXd prevStepPose = initialPose;

    // For each step
    double accum_t = 0.0;
    for (int i = 0; i < steps.size(); i++) {
        const Step& s = steps[i];

        // Case 1. The accumulated time is within the pause time
        accum_t += s.pause;
        if (t <= accum_t) {
            return prevStepPose;
        }

        // Case 2. The accumulated time is within the duration
        if (t <= accum_t + s.duration) {
            // return s.targetpose;

            double t_step = t - accum_t;
            double w = t_step / s.duration;
            Eigen::VectorXd pose = ((1 - w) * prevStepPose) + (w * s.targetpose);
            return pose;
        }
        accum_t += s.duration;
        // Update prevStepPose
        prevStepPose = s.targetpose;
    }

    // If we go though all the motion, 
    return steps[ steps.size() - 1 ].targetpose;
}

void Motion::printSteps() {
    for (int i = 0; i < steps.size(); i++) {
        const Step& s = steps[i];
        std::stringstream sin;
        sin << "Step " << i << ". ";
        sin << "duration = " << s.duration << " : pose = ";
        for (int j = 0; j < s.targetpose.size(); j++) {
            sin << s.targetpose(j) << " ";
        }
        sin << " (" << s.targetpose.size() << ")";
        LOG(INFO) << sin.str();
    }    
}

// struct Motion ends
////////////////////////////////////////////////////////////




} // namespace robot
} // namespace bioloidgp



