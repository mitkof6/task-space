#include "TaskBasedTorqueController.h"
#include <OpenSim/Actuators/CoordinateActuator.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

TaskBasedTorqueController::TaskBasedTorqueController(
    const ControlStrategy& controlStrategy)
    : controlStrategy(controlStrategy) {
}

void TaskBasedTorqueController::printResults(string prefix, string dir) {
    appliedForces.print(dir + "/" + prefix + "_TaskBasedTorqueController.sto");
}

void TaskBasedTorqueController::computeControls(const State& s,
    Vector& controls) const {
    // evaluate control strategy
    auto  forces = controlStrategy(s);
    appliedForces.append(s.getTime(), forces.size(), &forces[0], true);
    // apply forces as controls to the CoordinateActuators
    Vector actControls(1, 0.0);
    for (int i = 0; i < getActuatorSet().getSize(); i++) {
        auto act = dynamic_cast<const CoordinateActuator*>(
            &getActuatorSet().get(i));
        SimTK_ASSERT(act, "TaskBasedTorqueController::computeControls dynamic cast failed");
        Coordinate* aCoord = act->getCoordinate();
        if (aCoord->isConstrained(s)) {
            actControls = 0.0;
        } else {
            actControls = forces[i];
        }
        getActuatorSet()[i].addInControls(actControls, controls);
    }
}

void TaskBasedTorqueController::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    // construct labels for the applied forces
    Array<string> storageLabels;
    storageLabels.append("time");
    // create an actuator for each generalized coordinate in the model
    // add these actuators to the model and set their indexes
    auto& cs = _model->getCoordinateSet();
    for (int i = 0; i < cs.getSize(); i++) {
        std::string name = cs.get(i).getName() + "_control";
        CoordinateActuator* actuator = NULL;
        if (_model->getForceSet().contains(name)) {
            actuator = (CoordinateActuator*) &_model->getForceSet().get(name);
        } else {
            actuator = new CoordinateActuator();
            actuator->setCoordinate(&cs.get(i));
            actuator->setName(name);
            // Since this object is creating these actuators for its
            // own devices, it should take ownership of them, so that when
            // the controller is removed, so are all the actuators it added.
            adoptSubcomponent(actuator);
            setNextSubcomponentInSystem(*actuator);
        }
        actuator->setOptimalForce(1.0);
        updActuators().adoptAndAppend(actuator);
        // append labels
        storageLabels.append(cs[i].getName());
    }
    // configure Storage
    appliedForces.setColumnLabels(storageLabels);
    appliedForces.reset(0);
}