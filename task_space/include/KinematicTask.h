#ifndef KINEMATIC_TASK_H
#define KINEMATIC_TASK_H

#include <string>
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

class KinematicTask : public ModelComponent {
    OpenSim_DECLARE_ABSTRACT_OBJECT(KinematicTask, ModelComponent);
 public:
    KinematicTask(std::string body, SimTK::Vec3 offset);
    virtual SimTK::Matrix calcTaskJacobian(const SimTK::State& s) = 0;
    virtual SimTK::Vector calcTaskBias(const SimTK::State& s) = 0;
 protected:
    std::string body;
    SimTK::Vec3 offset;
};

class PositionTask : public KinematicTask {
    OpenSim_DECLARE_CONCRETE_OBJECT(PositionTask, KinematicTask);
 public:
    PositionTask(std::string body, SimTK::Vec3 offset);
    SimTK::Matrix calcTaskJacobian(const SimTK::State& s) override;
    SimTK::Vector calcTaskBias(const SimTK::State& s) override;
};

}

#endif
