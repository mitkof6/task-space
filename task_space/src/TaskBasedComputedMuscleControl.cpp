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
    cout << "Control Strategy: " << tau << endl;
    // calculate moment arm
    auto R = calcMomentArm(s, *_model);

    // calculate max force for activation = 1
    auto Fmax = calcMaxActuatorForce(s, *_model);

    // perform optimization
    try {
        target->prepareToOptimize(tau, R, Fmax);
        optimizer->optimize(activations);
    } catch (const SimTK::Exception::Base& ex) {
        cout << "OPTIMIZATION FAILED..." << endl;
        cout << ex.getMessage() << endl;
    }
    target->evaluateObjective(s, activations);

    // update controls
    controls += activations;
    controlStorage.append(s.getTime(), controls.size(), &controls[0]);
}

void TaskBasedComputedMuscleControl::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // create a reserve actuator for each generalized coordinate in the model
    // add these actuators to the model and set their indexes
    int numActuators = model.getActuators().getSize();
    int numReserveActuators = 0;
    auto& cs = model.getCoordinateSet();
    //for (int i = 0; i < cs.getSize(); i++) {
    //    std::string name = cs.get(i).getName() + "_reserve";
    //    CoordinateActuator* actuator = NULL;
    //    if (model.getForceSet().contains(name)) {
    //        actuator = (CoordinateActuator*) &model.getForceSet().get(name);
    //    } else {
    //        actuator = new CoordinateActuator();
    //        actuator->setCoordinate(&cs.get(i));
    //        actuator->setName(name);
    //        // since this object is creating these actuators for its own
    //        // devices, it should take ownership of them, so that when the
    //        // controller is removed, so are all the actuators it added
    //        adoptSubcomponent(actuator);
    //        setNextSubcomponentInSystem(*actuator);
    //        numReserveActuators++;
    //    }
    //    actuator->setOptimalForce(1.0);
    //    updActuators().adoptAndAppend(actuator);
    //}

    // construct labels for the actuators
    Array<string> storageLabels;
    storageLabels.append("time");
    auto& act = model.getActuators();
    for (int i = 0; i < act.getSize(); i++) {
        storageLabels.append(act[i].getName());
    }
    for (int i = 0; i < getActuatorSet().getSize(); i++) {
        storageLabels.append(getActuatorSet()[i].getName());
    }
    // configure Storage
    controlStorage.setColumnLabels(storageLabels);
    controlStorage.reset(0);

    // configure optimization target
    MuscleOptimizationTarget::OptimizationParameters targetParameters;
    targetParameters.numActuators = numActuators;
    targetParameters.numResidualActuators = numReserveActuators;
    targetParameters.numConstraints = cs.getSize();
    target = new MuscleOptimizationTarget(targetParameters);

    // parameter bounds
    Vector lowerBounds(numActuators + numReserveActuators);
    Vector upperBounds(numActuators + numReserveActuators);
    for (int i = 0; i < numActuators + numReserveActuators; i++) {
        if (i < numActuators) {
            lowerBounds[i] = 0;
            upperBounds[i] = 1;
        } else {
            /*lowerBounds[i] = -SimTK::Infinity;
            upperBounds[i] = SimTK::Infinity;*/
            lowerBounds[i] = -targetParameters.maxResidualForce;
            upperBounds[i] = targetParameters.maxResidualForce;
        }
    }
    target->setNumParameters(numActuators + numReserveActuators);
    target->setNumEqualityConstraints(targetParameters.numConstraints);
    target->setParameterLimits(lowerBounds, upperBounds);
    activations = Vector(target->getNumParameters(), 0.0);

    // optimization
    optimizer = new Optimizer(target.getRef(), OptimizerAlgorithm::InteriorPoint);
    optimizer->setConvergenceTolerance(1E-4);
    optimizer->setConstraintTolerance(1E-4);
    optimizer->setMaxIterations(500);
    optimizer->setLimitedMemoryHistory(500);
    optimizer->setDiagnosticsLevel(3);
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

    cout << "Optimization Statistics" << endl
        << "\ttime.................: " << s.getTime() << endl
        << "\tactuators............: " << actuatorObj << endl
        << "\tresiduals............: " << residualActuatorObj << endl
        << "\tconstraint violation.: " << sqrt(~constraints * constraints)
        << endl;
}

int MuscleOptimizationTarget::objectiveFunc(const Vector& x, bool newPar,
                                            Real& f) const {
    f = 0.0;
    // actuators [0, 1]
    for (int i = 0; i < parameters.numActuators; i++) {
        f += parameters.alpha * pow(x[i], parameters.activationExponent)
            / parameters.numActuators;
    }
    // residual actuators [-max, max]
    for (int i = parameters.numActuators; i < getNumParameters(); i++) {
        f +=
            parameters.beta *
            pow(abs(x[i]) / parameters.maxResidualForce,
                parameters.activationExponent) /
            parameters.numResidualActuators;
    }
    return 0;
}

int MuscleOptimizationTarget::gradientFunc(const Vector& x, bool newPar,
                                           Vector& gradient) const {
    // actuators [0, 1]
    for (int i = 0; i < parameters.numActuators; i++) {
        gradient[i] =
            parameters.alpha * parameters.activationExponent *
            pow(x[i], parameters.activationExponent - 1) /
            parameters.numActuators;
    }
    // residual actuators
    for (int i = parameters.numActuators; i < getNumParameters(); i++) {
        if (x[i] > 0) {
            gradient[i] =
                parameters.beta * parameters.activationExponent *
                pow(abs(x[i]) / parameters.maxResidualForce, parameters.activationExponent - 1) /
                (parameters.numResidualActuators * parameters.maxResidualForce);
        } else {
            gradient[i] =
                -parameters.beta * parameters.activationExponent *
                pow(abs(x[i]) / parameters.maxResidualForce, parameters.activationExponent - 1) /
                (parameters.numResidualActuators * parameters.maxResidualForce);
        }
    }
    return 0;
}

int MuscleOptimizationTarget::constraintFunc(const Vector& x, bool newPar,
                                             Vector& constraints) const {
    // a = activations
    Vector a = x(0, parameters.numActuators);
    //cout << "a " << a << endl;
    // tau = R * Fmax * a
    Vector actuatorTorque = R * Fmax.elementwiseMultiply(a);
    //cout << "M " << actuatorTorque << endl;
    // get residual actuator
    Vector residualTorques(tau.size(), 0.0);
    if (parameters.numResidualActuators != 0) {
        residualTorques = x(parameters.numActuators,
                            parameters.numResidualActuators);
    }
    //cout << "R " << residualTorques << endl;
    // R * Fmax * a + tau_res - tau_des = 0
    constraints = actuatorTorque + residualTorques - tau;
    //cout << "C " << constraints << endl;

    return 0;
}

int MuscleOptimizationTarget::constraintJacobian(const Vector& x, bool newPar,
                                                 Matrix& jac) const {
    return 0;
}