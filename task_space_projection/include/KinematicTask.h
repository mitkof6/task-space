#ifndef KINEMATIC_TASK_H
#define KINEMATIC_TASK_H

#include <string>
#include <OpenSim/OpenSim.h>

class KinematicTask {
    std::string body;
    SimTK::Vec3 offset;
 public:
    KinematicTask();

};

#endif
