/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "MotorMap.h"

#include <algorithm>
#include <tinyxml2.h>

#include "utils/CppCommon.h"

namespace bioloidgp {
namespace robot {

////////////////////////////////////////////////////////////
// struct Motor implementation
struct Motor {
    Motor() : index(-1), id(-1), name("") {}
    Motor(int _index, int _id, std::string& _name)
        : index(_index)
        , id(_id)
        , name(_name)
        {
            angleAtMin = -(5.0 / 6.0) * UTILS_PI;
            angleAtMax =  (5.0 / 6.0) * UTILS_PI;
        }
    int index;
    int id;
    std::string name;

    double angleAtMin;
    double angleAtMax;

    void swapMinMaxAngles() {
        std::swap(angleAtMin, angleAtMax);
    }

    void addOffsetToMinMaxAngles(double o) {
        angleAtMin += o;
        angleAtMax += o;
    }

    double fromMotorMap(double mv) {
        // 0 --> -150 deg
        // 1024 --> 150 deg
        double w = (mv / 1024.0);
        double angle = w * (angleAtMax - angleAtMin) + angleAtMin;
        return angle;
    }

    double toMotorMap(double v) {
        // 0 --> -150 deg
        // 1024 --> 150 deg
        double w = (v - angleAtMin) / (angleAtMax - angleAtMin);
        double angle = w * 1024.0;
        return angle;
    }


};
// class Motor ends
//////////////////////////////////////////////////////////// 



////////////////////////////////////////////////////////////
// class MotorMap implementation
MotorMap::MotorMap(int _maxMotorID, int _numFullDimensions)
    : mMaxMotorIDs(_maxMotorID)
    , mNumFullDimensions(_numFullDimensions)
{
}

MotorMap::~MotorMap() {
}


bool MotorMap::load(const char* const filename) {
    LOG(INFO) << "load motormap data from [" << filename << "]";
    // Read XML
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result
        = doc.LoadFile(filename);
        // = doc.LoadFile(DART_DATA_PATH"xml/damageReduce/scene.xml" );
    if (result != tinyxml2::XML_NO_ERROR) {
        doc.PrintError();
        LOG(ERROR) << "ErrorStr1: " << doc.GetErrorStr1();
        LOG(ERROR) << "ErrorStr2: " << doc.GetErrorStr2();
        return false;
    }
    LOG(INFO) << "read XML OK";

    tinyxml2::XMLElement* xnRobot = doc.FirstChildElement("robot");
    for (tinyxml2::XMLElement* xnMotor = xnRobot->FirstChildElement("motor");
         xnMotor; xnMotor = xnMotor->NextSiblingElement("motor")) {
        int index = xnMotor->IntAttribute("index");
        int id    = xnMotor->IntAttribute("id");
        std::string name = xnMotor->Attribute("name");
        // Extra
        bool swap = false;
        double offset = 0.0;
        xnMotor->QueryBoolAttribute("swap", &swap);
        xnMotor->QueryDoubleAttribute("offset", &offset);

        Motor* m = new Motor(index, id, name);
        if (swap) {
            m->swapMinMaxAngles();
        }
        m->addOffsetToMinMaxAngles(offset);

        motors.push_back(m);
        LOG(INFO) << index << " " << id << " " << name
                  << " {" << m->angleAtMin << " " << m->angleAtMax << "} "
                  << "(offset = " << offset << ", "
                  << "swap = " << swap
                  << ")"; 

    }
    LOG(INFO) << "read " << numMotors() << " motor data successfully.";

    return true;
}

int MotorMap::findMotorIndex(int motorID) const {
    FOREACH(Motor* m, motors) {
        if (m->id == motorID) {
            return m->index;
        }
    }
    return -1;
}

int MotorMap::findMotorIndex(const char* const motorName) const {
    FOREACH(Motor* m, motors) {
        if (m->name == motorName) {
            return m->index;
        }
    }
    return -1;
}

Eigen::VectorXd MotorMap::toMotorMapVector(const Eigen::VectorXd& v) const {
    Eigen::VectorXd mtv = Eigen::VectorXd::Zero(numMotors());
    FOREACH(Motor* m, motors) {
        mtv(m->id) = m->toMotorMap( v(m->index) );
    }
    return mtv;
}

Eigen::VectorXd MotorMap::fromMotorMapVector(const Eigen::VectorXd& mtv) const {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(numFullDimensions());
    FOREACH(Motor* m, motors) {
        v(m->index) = m->fromMotorMap( mtv(m->id) );
    }
    return v;
}

// class MotorMap ends
////////////////////////////////////////////////////////////



} // namespace robot
} // namespace bioloidgp



