#include "TaskBasedComputedMuscleControl.h"
#include <OpenSim/Actuators/CoordinateActuator.h>

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
    Matrix R(momentArm.size(), momentArm[0].size(), &momentArm[0][0]);
    return ~R;
}

/******************************************************************************/

TaskBasedComputedMuscleControl::TaskBasedComputedMuscleControl(
    const ControlStrategy & controlStrategy) : controlStrategy(controlStrategy) {
}

void TaskBasedComputedMuscleControl::printResults(string prefix, string dir) {
    controlStorage.print(dir + "/" + prefix +
        "_TaskBasedComputedMuscleControl.sto");
}

void TaskBasedComputedMuscleControl::computeControls(const State& s,
    Vector& controls) const {
    // evaluate control strategy
    auto forces = controlStrategy(s);

    // calculate moment arm
    auto R = calcMomentArm(s, *_model);

    // calculate max force for activation = 1
    auto Fmax = calcMaxActuatorForce(s, *_model);

    // perform optimization

    // update controls
    //controls += excitations;
    controlStorage.append(s.getTime(), controls.size(), &controls[0]);
}

void TaskBasedComputedMuscleControl::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // create a reserve actuator for each generalized coordinate in the model
    // add these actuators to the model and set their indexes
    int numActuators = _model->getActuators().getSize();
    int numReserveActuators = 0;
    auto& cs = _model->getCoordinateSet();
    for (int i = 0; i < cs.getSize(); i++) {
        std::string name = cs.get(i).getName() + "_reserve";
        CoordinateActuator* actuator = NULL;
        if (_model->getForceSet().contains(name)) {
            actuator = (CoordinateActuator*) &_model->getForceSet().get(name);
        } else {
            actuator = new CoordinateActuator();
            actuator->setCoordinate(&cs.get(i));
            actuator->setName(name);
            // since this object is creating these actuators for its own
            // devices, it should take ownership of them, so that when the
            // controller is removed, so are all the actuators it added
            adoptSubcomponent(actuator);
            setNextSubcomponentInSystem(*actuator);
            numReserveActuators++;
        }
        actuator->setOptimalForce(1.0);
        updActuators().adoptAndAppend(actuator);
    }

    // construct labels for the actuators
    Array<string> storageLabels;
    storageLabels.append("time");
    auto& act = _model->getActuators();
    for (int i = 0; i < act.getSize(); i++) {
        storageLabels.append(act[i].getName());
    }
    // configure Storage
    controlStorage.setColumnLabels(storageLabels);
    controlStorage.reset(0);

    // optimization target
    MuscleOptimizationTarget::OptimizationParameters targetParameters;
    targetParameters.numActuators = numActuators;
    targetParameters.numResidualActuators = numReserveActuators;
    target = new MuscleOptimizationTarget(targetParameters);

    // optimization
    optimizer = new Optimizer(*target, OptimizerAlgorithm::InteriorPoint);
    optimizer->setConvergenceTolerance(1E-4);
    optimizer->setConstraintTolerance(1E-4);
    optimizer->setMaxIterations(1000);
    optimizer->setLimitedMemoryHistory(1000);
    optimizer->setDiagnosticsLevel(0);
    //optimizer->setAdvancedBoolOption("warm_start", true);
    //optimizer->setAdvancedRealOption("obj_scaling_factor", 1);
    //optimizer->setAdvancedRealOption("nlp_scaling_max_gradient", 1);
    optimizer->useNumericalGradient(false);
    optimizer->useNumericalJacobian(true);
}

/******************************************************************************/

MuscleOptimizationTarget::MuscleOptimizationTarget(OptimizationParameters parameters)
    : OptimizerSystem(parameters.numActuators + parameters.numResidualActuators),
    parameters(parameters) {
}

void MuscleOptimizationTarget::prepareToOptimize(const Vector& desiredTau,
    const Matrix& momentArm, const Vector& maxForce) {
    tau = desiredTau;
    R = momentArm;
    Fmax = maxForce;
}

void MuscleOptimizationTarget::evaluateObjective(const State& s,
    const Vector& x) {
    double actuatorObj;
    Vector temp(getNumParameters(), 0.0);
    temp(0, parameters.numActuators) = x(0, parameters.numActuators);
    objectiveFunc(temp, true, actuatorObj);

    double residualActuatorObj;
    temp = Vector(getNumParameters(), 0.0);
    temp(parameters.numActuators, parameters.numResidualActuators) =
        x(parameters.numActuators, parameters.numResidualActuators);
    objectiveFunc(temp, true, residualActuatorObj);

    Vector constraints(getNumConstraints());
    constraintFunc(x, true, constraints);

    cout
        << "Time " << setw(24) << s.getTime() << endl
        << "\tactuator: " << setw(20) << actuatorObj << endl
        << "\tresidual: " << setw(20) << residualActuatorObj << endl
        << "\tconstraint violation: " << setw(20)
        << sqrt(~constraints * constraints) << endl;
}

int MuscleOptimizationTarget::objectiveFunc(const Vector& x, bool newPar,
    Real& f) const {
    f = 0.0;
    // actuators
    for (int i = 0; i < parameters.numActuators; i++) {
        f += pow(abs(x[i]), parameters.activationExponent);
    }
    // residual actuators
    for (int i = parameters.numActuators; i < getNumParameters(); i++) {
        f += parameters.gamma * pow(abs(x[i]) / parameters.maxResidualForce, 2);
    }
    return 0;
}

int MuscleOptimizationTarget::gradientFunc(const Vector& x, bool newPar,
    Vector& gradient) const {
    // actuators
    for (int i = 0; i < parameters.numActuators; i++) {
        if (x[i] > 0) {
            gradient[i] = parameters.activationExponent * pow(abs(x[i]),
                parameters.activationExponent - 1);
        } else {
            gradient[i] = -parameters.activationExponent  * pow(abs(x[i]),
                parameters.activationExponent - 1);
        }
    }
    // residual actuators
    for (int i = parameters.numActuators; i < getNumParameters(); i++) {
        if (x[i] > 0) {
            gradient[i] = parameters.gamma * 2 * abs(x[i]) /
                parameters.maxResidualForce;
        } else {
            gradient[i] = -parameters.gamma * 2 * abs(x[i]) /
                parameters.maxResidualForce;
        }
    }
    return 0;
}

int MuscleOptimizationTarget::constraintFunc(const Vector& x, bool newPar,
    Vector& constraints) const {
    // a = activations
    Vector a = x(0, parameters.numActuators);

    // tau = R * Fmax * a
    Vector actuatorTorque = R * Fmax.elementwiseMultiply(a);

    // get residual actuator
    Vector residualTorques(tau.size(), 0.0);
    if (parameters.numResidualActuators != 0) {
        residualTorques = x(parameters.numActuators,
            parameters.numResidualActuators);
    }

    // R * Fmax * a + tau_res - tau_des = 0
    constraints = actuatorTorque + residualTorques - tau;
    return 0;
}

int MuscleOptimizationTarget::constraintJacobian(const Vector& x, bool newPar,
    Matrix& jac) const {
    return 0;
}