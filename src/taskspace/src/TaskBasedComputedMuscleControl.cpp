#include "TaskBasedComputedMuscleControl.h"
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/Adapters.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/******************************************************************************/

Vector calcMaxActuatorForce(const State& s, const Model& model) {
    vector<double> optimalForce;
    auto& actuatorSet = model.getActuators();
    for (int i = 0; i < actuatorSet.getSize(); i++) {
        auto pathActuator = dynamic_cast<const PathActuator*>(&actuatorSet[i]);
        if (pathActuator) {
            double force;
            auto muscle = dynamic_cast<const Muscle*>(&actuatorSet[i]);
            if (muscle) {
                force = muscle->getMaxIsometricForce();
            } else {
                force = pathActuator->getOptimalForce();
            }
            optimalForce.push_back(force);
        }
    }
    return Vector(optimalForce.size(), &optimalForce[0]);
}

Matrix calcMomentArm(const State& s, const Model& model) {
    auto& coordinateSet = model.getCoordinateSet();
    auto& actuatorSet = model.getActuators();
    vector<vector<double> > momentArm;
    for (int i = 0; i < actuatorSet.getSize(); i++) {
        auto pathActuator = dynamic_cast<const PathActuator*>(&actuatorSet[i]);
        if (pathActuator) {
            vector<double> column;
            for (int j = 0; j < coordinateSet.getSize(); j++) {
                column.push_back(
                    pathActuator->computeMomentArm(s, coordinateSet[j]));
            }
            momentArm.push_back(column);
        }
    }
    // convert to Matrix
    Matrix R(momentArm[0].size(), momentArm.size());
    for (unsigned int m = 0; m < momentArm.size(); m++) {
        for (unsigned int n = 0; n < momentArm[0].size(); n++) {
            R[n][m] = momentArm[m][n];
        }
    }
    return R;
}

/******************************************************************************/

TaskBasedComputedMuscleControl::TaskBasedComputedMuscleControl(
    const ControlStrategy & controlStrategy)
    : controlStrategy(controlStrategy) {
}

void TaskBasedComputedMuscleControl::printResults(string prefix, string dir) {
    STOFileAdapter::write(controlsDataTable,
                          dir + "/" + prefix +
                          "_TaskBasedComputedMuscleControl.sto");
}

void TaskBasedComputedMuscleControl::computeControls(const State& s,
                                                     Vector& controls) const {
    // evaluate control strategy
    auto tau = controlStrategy(s);
    // calculate moment arm
    auto R = calcMomentArm(s, *_model);
    // calculate max force for activation = 1
    auto Fmax = calcMaxActuatorForce(s, *_model);

    // optimization
    Vector tauReserve(getActuatorSet().getSize(), 0.0);
    Vector activations = getInitialActivations(s);
    try {
        target->prepareToOptimize(tau, R, Fmax);
        optimizer->optimize(activations);
    } catch (const SimTK::Exception::Base& ex) {
        cout << "Optimization failed..." << endl;
        cout << ex.getMessage() << endl;
        cout << "Reserve actuators will be applied" << endl;
        // cout << "Control strategy: " << tau << endl;
        target->constraintFunc(activations, true, tauReserve);
    }
    target->evaluateObjective(s, activations);
    // correspond residuals to reserve actuators in controls vector
    for (int i = 0; i < getActuatorSet().getSize(); i++) {
        getActuatorSet()[i].addInControls(Vector(1, tauReserve[i]), controls);
    }
    // correspond activations in controls vector
    for (int i = 0; i < _model->getActuators().getSize(); i++) {
        _model->getActuators()[i].addInControls(Vector(1, activations[i]),
                                                controls);
    }

    // append controls in data table
    controlsDataTable.appendRow(s.getTime(),
                                controls.transpose().getAsRowVector());
}

void TaskBasedComputedMuscleControl::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // create a reserve actuator for each generalized coordinate in the model
    // add these actuators to the model and set their indexes
    int numActuators = model.getActuators().getSize();
    auto& cs = model.getCoordinateSet();
    for (int i = 0; i < cs.getSize(); i++) {
        std::string name = cs.get(i).getName() + "_reserve";
        CoordinateActuator* actuator = NULL;
        if (model.getForceSet().contains(name)) {
            actuator = (CoordinateActuator*) &model.getForceSet().get(name);
        } else {
            actuator = new CoordinateActuator();
            actuator->setCoordinate(&cs.get(i));
            actuator->setName(name);
            // since this object is creating these actuators for its own
            // devices, it should take ownership of them, so that when the
            // controller is removed, so are all the actuators it added
            adoptSubcomponent(actuator);
            setNextSubcomponentInSystem(*actuator);
        }
        actuator->setOptimalForce(1.0);
        updActuators().adoptAndAppend(actuator);
    }

    // construct labels for the actuators
    vector<string> columnLabels;
    // reserve actuators are listed first in the controls vector
    for (int i = 0; i < getActuatorSet().getSize(); i++) {
        columnLabels.push_back(getActuatorSet()[i].getName());
    }
    // append model actuators after reserve actuators
    auto& act = model.getActuators();
    for (int i = 0; i < act.getSize(); i++) {
        columnLabels.push_back(act[i].getName());
    }
    controlsDataTable.setColumnLabels(columnLabels);

    // configure optimization target
    MuscleOptimizationTarget::OptimizationParameters targetParameters;
    targetParameters.numActuators = numActuators;
    targetParameters.numConstraints = cs.getSize();
    targetParameters.activationExponent = 2;
    target = new MuscleOptimizationTarget(targetParameters);
    Vector lowerBounds(targetParameters.numActuators);
    Vector upperBounds(targetParameters.numActuators);
    for (int i = 0; i < targetParameters.numActuators; i++) {
        lowerBounds[i] = 0;
        upperBounds[i] = 1;
    }
    target->setNumParameters(targetParameters.numActuators);
    target->setNumEqualityConstraints(targetParameters.numConstraints);
    target->setParameterLimits(lowerBounds, upperBounds);

    // optimization
    optimizer = new Optimizer(target.getRef(),
                              OptimizerAlgorithm::InteriorPoint);
    optimizer->setConvergenceTolerance(1E-4);
    optimizer->setConstraintTolerance(1E-4);
    //optimizer->setMaxIterations(500);
    //optimizer->setLimitedMemoryHistory(100);
    optimizer->setDiagnosticsLevel(0);
    //optimizer->setAdvancedBoolOption("warm_start", true);
    //optimizer->setAdvancedRealOption("obj_scaling_factor", 1);
    //optimizer->setAdvancedRealOption("nlp_scaling_max_gradient", 1);
    optimizer->useNumericalGradient(false);
    optimizer->useNumericalJacobian(false);
}

Vector TaskBasedComputedMuscleControl::getInitialActivations(const State& s)
const {
    // the integrator may go back in time so remove the future rows
    double t = s.getTime();
    while (controlsDataTable.getNumRows() != 0 &&
           t <= controlsDataTable.getIndependentColumn().back()) {
        controlsDataTable.removeRowAtIndex(controlsDataTable.getNumRows() - 1);
    }
    // if storage is empty return zero activations
    int nActivations = target->getNumParameters();
    if (controlsDataTable.getNumRows() == 0) {
        return Vector(nActivations, 0.0);
    }
    // else return the last row (note that activations are the last m elements)
    auto row = controlsDataTable.getRowAtIndex(controlsDataTable.getNumRows()
                                               - 1);
    Vector activations(nActivations, 0.0);
    for (int i = 0; i < nActivations; i++) {
        activations[i] = row.getAnyElt(0, row.size() - nActivations + i);
    }
    return activations;
}

/******************************************************************************/

MuscleOptimizationTarget::MuscleOptimizationTarget(
    OptimizationParameters parameters)
    : OptimizerSystem(), parameters(parameters) {
    SimTK_ASSERT_ALWAYS(parameters.activationExponent > 0,
                        "MuscleOptimizationTarget: activationExponent > 0");
}

void MuscleOptimizationTarget::prepareToOptimize(const Vector& desiredTau,
                                                 const Matrix& momentArm,
                                                 const Vector& maxForce) {
    tau = desiredTau;
    R = momentArm;
    Fmax = maxForce;
}

void MuscleOptimizationTarget::evaluateObjective(const State& s,
                                                 const Vector& x) {
    double actuatorObj;
    objectiveFunc(x, true, actuatorObj);

    Vector constraints(getNumConstraints());
    constraintFunc(x, true, constraints);

    cout << "Optimization Statistics" << endl
        << "\ttime.................: " << s.getTime() << endl
        << "\tactuators............: " << actuatorObj << endl
        << "\tconstraint violation.: " << sqrt(~constraints * constraints)
        << endl;
}

int MuscleOptimizationTarget::objectiveFunc(const Vector& x, bool newPar,
                                            Real& f) const {
    f = 0.0;
    for (int i = 0; i < getNumParameters(); i++) {
        f += pow(abs(x[i]), parameters.activationExponent);
    }
    return 0;
}

int MuscleOptimizationTarget::gradientFunc(const Vector& x, bool newPar,
                                           Vector& gradient) const {
    for (int i = 0; i < getNumParameters(); i++) {
        if (x[i] > 0) {
            gradient[i] = parameters.activationExponent *
                pow(abs(x[i]), parameters.activationExponent - 1);
        } else {
            gradient[i] = -parameters.activationExponent *
                pow(abs(x[i]), parameters.activationExponent - 1);
        }
    }
    return 0;
}

int MuscleOptimizationTarget::constraintFunc(const Vector& x, bool newPar,
                                             Vector& constraints) const {
    // tau = R * Fmax * a
    Vector actuatorTorque = R * Fmax.elementwiseMultiply(x);
    // tau_des - tau = 0
    constraints = tau - actuatorTorque;
    return 0;
}

int MuscleOptimizationTarget::constraintJacobian(const Vector& x, bool newPar,
                                                 Matrix& jac) const {
    // d/da = -R Fmax dA/da
    for (int i = 0; i < jac.nrow(); i++) {
        for (int j = 0; j < jac.ncol(); j++) {
            jac[i][j] = -R(i, j) * Fmax(j);
        }
    }
    return 0;
}