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
    // convert to Matrix
    Matrix R(momentArm[0].size(), momentArm.size());
    for (int m = 0; m < momentArm.size(); m++) {
        for (int n = 0; n < momentArm[0].size(); n++) {
            R[n][m] = momentArm[m][n];
        }
    }
    return R;
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
    auto tau = controlStrategy(s);
    //cout << "Control Strategy: " << tau << endl;
    // calculate moment arm
    auto R = calcMomentArm(s, *_model);
    //cout << R << endl;

    // calculate max force for activation = 1
    auto Fmax = calcMaxActuatorForce(s, *_model);
    //cout << Fmax << endl;

    // perform optimization
    Vector tauReserve(getActuatorSet().getSize(), 0.0);
    try {
        target->prepareToOptimize(tau, R, Fmax);
        optimizer->optimize(activations);
    } catch (const SimTK::Exception::Base& ex) {
        cout << "OPTIMIZATION FAILED..." << endl;
        cout << ex.getMessage() << endl;
        //activations = previouslyValidActivations;
        target->constraintFunc(activations, true, tauReserve);
    }
    target->evaluateObjective(s, activations);

    // correspond residuals to reserve actuators
    for (int i = 0; i < getActuatorSet().getSize(); i++) {
        getActuatorSet()[i].addInControls(Vector(1, tauReserve[i]), controls);
    }
    // correspond activations to controls
    for (int i = 0; i < _model->getActuators().getSize(); i++) {
        _model->getActuators()[i].addInControls(Vector(1, activations[i]),
                                                controls);
    }

    // store
    //cout << controls << endl;
    controlStorage.crop(0, s.getTime());
    controlStorage.append(s.getTime(), controls.size(), &controls[0]);
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
    Array<string> storageLabels;
    storageLabels.append("time");
    // reserve actuators are listed first in the controls vector
    for (int i = 0; i < getActuatorSet().getSize(); i++) {
        storageLabels.append(getActuatorSet()[i].getName());
    }
    // model actuators
    auto& act = model.getActuators();
    for (int i = 0; i < act.getSize(); i++) {
        storageLabels.append(act[i].getName());
    }
    // configure Storage
    controlStorage.setColumnLabels(storageLabels);
    controlStorage.reset(0);

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
    activations = Vector(target->getNumParameters(), 0.0);

    // optimization
    optimizer = new Optimizer(target.getRef(), OptimizerAlgorithm::InteriorPoint);
    optimizer->setConvergenceTolerance(1E-4);
    optimizer->setConstraintTolerance(1E-4);
    optimizer->setMaxIterations(1500);
    optimizer->setLimitedMemoryHistory(1500);
    optimizer->setDiagnosticsLevel(0);
    //optimizer->setAdvancedBoolOption("warm_start", true);
    //optimizer->setAdvancedRealOption("obj_scaling_factor", 1);
    //optimizer->setAdvancedRealOption("nlp_scaling_max_gradient", 1);
    optimizer->useNumericalGradient(false);
    optimizer->useNumericalJacobian(true);
}

/******************************************************************************/

MuscleOptimizationTarget::MuscleOptimizationTarget(OptimizationParameters parameters)
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
    //cout << "M " << actuatorTorque << endl;
    // R * Fmax * a + tau_res - tau_des = 0
    constraints = tau - actuatorTorque;
    //cout << "C " << constraints << endl;
    return 0;
}

int MuscleOptimizationTarget::constraintJacobian(const Vector& x, bool newPar,
                                                 Matrix& jac) const {
    throw logic_error("Not implemented yet");
    return 0;
}