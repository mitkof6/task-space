#include "TaskBasedForce.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

TaskBasedForce::TaskBasedForce(const ControlStrategy& controlStrategy)
    : controlStrategy(controlStrategy) {
    // TODO when evaluating all forces
    // taskManager->updDynamicCompensator().addExcludedForces(this);
}

void TaskBasedForce::printResults(std::string prefix, std::string dir) {
    appliedForces.print(dir + "/" + prefix + "_TaskBasedForce.sto");
}

void TaskBasedForce::computeForce(const State& s,
				  Vector_<SpatialVec>& bodyForces,
                                  Vector & generalizedForces) const {
    auto forces = controlStrategy(s);
    appliedForces.append(s.getTime(), forces.size(), &forces[0], true);
    generalizedForces += forces;
}

void TaskBasedForce::extendInitStateFromProperties(State& s) const {
    Super::extendInitStateFromProperties(s);
    // construct labels for the applied forces
    Array<string> labels;
    labels.append("time");
    const auto& cs = _model->getCoordinateSet();
    for (int i = 0; i < cs.getSize(); i++) {
        labels.append(cs[i].getName());
    }
    appliedForces.setColumnLabels(labels);
    appliedForces.reset(0);
}
