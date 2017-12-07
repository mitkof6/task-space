#include "KinematicTask.h"

using namespace OpenSim;

KinematicTask::KinematicTask(std::string body, SimTK::Vec3 offset)
    : body(body), offset(offset)
{

}

PositionTask::PositionTask(std::string body, SimTK::Vec3 offset)
    : KinematicTask(body, offset)
{

}

SimTK::Matrix PositionTask::calcTaskJacobian(const SimTK::State& s) {

    SimTK::Matrix J;
    return J;
}

SimTK::Vector PositionTask::calcTaskBias(const SimTK::State& s) {

    SimTK::Vector JdotQdot;
    return JdotQdot;
}
