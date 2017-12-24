#include "TaskBasedForce.h"
#include "TaskManager.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

TaskBasedForce::TaskBasedForce(TaskManager* taskManager)
    : taskManager(taskManager) {
    // taskManager->updDynamicCompensator().addExcludedForces(this);
}

void TaskBasedForce::printResults(std::string prefix, std::string dir) {
    forcesSto.print(dir + "/" + prefix + "_TaskForces.sto");
    // taskDynamics->printResults(prefix, dir);
}

void TaskBasedForce::computeForce(const State& s, Vector_<SpatialVec>& bodyForces,
                                  Vector & generalizedForces) const {
    Vector controls = taskManager->calcTaskTorques(s);
    forcesSto.append(s.getTime(), controls.size(), &controls[0], true);
    generalizedForces += controls;
}

void TaskBasedForce::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);

    Array<string> labels;
    labels.append("time");
    const auto& cs = _model->getCoordinateSet();
    for (int i = 0; i < cs.getSize(); i++) {
        labels.append(cs[i].getName());
    }

    forcesSto.reset(0);
    forcesSto.setColumnLabels(labels);
}
