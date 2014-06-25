/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "Motion.h"
#include <sstream>
#include <tinyxml2.h>
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
    cout << "step. duration = " << s.duration << " : "
         << s.targetpose.transpose()
         << " (" << s.targetpose.size() << ")"
         << endl;
}

// struct Step ends
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// struct Motion implementation
Motion::Motion(int _dim)
    : dim(_dim)
    , stepIndex(-1)
{
}

bool Motion::load(const char* const filename) {
    cout << "load motion from [" << filename << "]" << endl;
    // Read XML
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result
        = doc.LoadFile(filename);
        // = doc.LoadFile(DART_DATA_PATH"xml/damageReduce/scene.xml" );
    if (result != tinyxml2::XML_NO_ERROR) {
        doc.PrintError();
        cerr << "ErrorStr1: " << doc.GetErrorStr1() << endl;
        cerr << "ErrorStr2: " << doc.GetErrorStr2() << endl;
        return false;
    }
    cout << "read XML OK" << endl;

    steps.clear();
    tinyxml2::XMLElement* xnMotion = doc.FirstChildElement("motion");
    for (tinyxml2::XMLElement* xnStep = xnMotion->FirstChildElement("step");
         xnStep; xnStep = xnStep->NextSiblingElement("step")) {
        Step s;
        parseStep(s, dim, xnStep);
        steps.push_back(s);
    }
    cout << FUNCTION_NAME() << " OK" << endl;
}

Eigen::VectorXd Motion::targetPose(double t) const {
    double accum_t = 0.0;
    for (int i = 0; i < steps.size(); i++) {
        const Step& s = steps[i];
        accum_t += s.duration;
        bool is_last = ((i + 1) == steps.size());
        if (t <= accum_t || is_last) {
            cout << t << " " << accum_t << " -> " << i << endl;
            return s.targetpose;
        }
    }
    // I thing it should be reached, but..
    return Eigen::VectorXd::Zero(dim);
}

// struct Motion ends
////////////////////////////////////////////////////////////




} // namespace robot
} // namespace bioloidgp



