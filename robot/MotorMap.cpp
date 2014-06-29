/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "MotorMap.h"

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
        }
    int index;
    int id;
    std::string name;
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
        Motor* m = new Motor(index, id, name);
        motors.push_back(m);
        LOG(INFO) << index << " " << id << " " << name; 
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
        mtv(m->id) = v(m->index);
    }
    return mtv;
}

Eigen::VectorXd MotorMap::fromMotorMapVector(const Eigen::VectorXd& mtv) const {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(numFullDimensions());
    FOREACH(Motor* m, motors) {
        v(m->index) = mtv(m->id);
    }
    return v;
}

// class MotorMap ends
////////////////////////////////////////////////////////////



} // namespace robot
} // namespace bioloidgp



