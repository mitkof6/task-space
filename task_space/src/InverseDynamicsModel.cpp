#include "InverseDynamicsModel.h"

using namespace OpenSim;
using namespace SimTK;

/**
 * Calculates the gravity contribution \f$ \tau_g \f$. We assume that
 * \f$ \tau_g \f$ is on the same side with the joint space accelerations
 * (i.e. \f$ M \ddot{q} + \tau_g = \tau \f$).
 */
Vector calcGravity(const State& s, const Model& model) {
    Vector g;
    model.getMatterSubsystem().multiplyBySystemJacobianTranspose(
        s, model.getGravityForce().getBodyForces(s), g);
    return -g;
}

/**
 * Calculates the Coriolis contribution \f$ \tau_c \f$. We assume that
 * \f$ \tau_c \f$ is on the same side with the joint space accelerations
 * (i.e. \f$ M \ddot{q} + \tau_c = \tau \f$).
 */
Vector calcCoriolis(const State& s, const Model& model) {
    Vector c;
    model.getMatterSubsystem().calcResidualForceIgnoringConstraints(
        s, Vector(0), Vector_<SpatialVec>(0), Vector(0), c);
    return c;
}

/**
*
*/
Model* workingModel;
Vector calcTotalForces(const State& s, const Model& model) {
    // create a working instance of the model, upon first entrance to this 
    // function
    static bool initialized;
    if (!initialized) {
        initialized = true;
        workingModel = model.clone();
        workingModel->updControllerSet().setSize(0);
        workingModel->setUseVisualizer(false);
        workingModel->initSystem();
    }
    // create working state from s and override model defaults 
    State workingState = s;
    workingState = workingModel->getWorkingState();
    // disable any actuators when computing the total force
    const Set<Actuator>& as = workingModel->getActuators();
    for (int i = 0; i < as.getSize(); i++) {
        as[i].setAppliesForce(workingState, false);
    }
    // generalized forces and torques should be accessed at a Dynamics stage
    workingModel->realizeDynamics(workingState);
    // get acting torques and body forces
    auto bodyForces = workingModel->getMultibodySystem()
        .getRigidBodyForces(workingState, Stage::Dynamics);
    auto generalizedForces = workingModel->getMultibodySystem()
        .getMobilityForces(workingState, Stage::Dynamics);
    // map body forces to joint forces
    Vector jointForces;
    workingModel->getMatterSubsystem()
        .multiplyBySystemJacobianTranspose(workingState, bodyForces, jointForces);
    // in our conviction we subtract their contribution
    return -1.0 * generalizedForces - jointForces;
}

Matrix calcM(const State& s, const Model& model) {
    Matrix M;
    model.getMatterSubsystem().calcM(s, M);
    return M;
}

Matrix calcMInv(const State& s, const Model& model) {
    Matrix MInv;
    model.getMatterSubsystem().calcMInv(s, MInv);
    return MInv;
}

Vector calcTotalGeneralizedForces(const State& s, const Model& model) {
    // compute all acting forces add Coriolis since they are not accounted
    return  calcCoriolis(s, model) + calcTotalForces(s, model);
    // compute only Coriolis and gravity
    //return calcCoriolis(s, model) + calcGravity(s, model);
}

Matrix calcConstraintJacobian(const State& s, const Model& model) {
    Matrix Phi;
    model.getMatterSubsystem().calcG(s, Phi);
    return Phi;
}

Vector calcConstraintBias(const State& s, const Model& model) {
    // calcBiasForAccelerationConstraints assumes P *udot - bias = 0 we must
    // invert the results so that P * udot = bias
    Vector b;
    model.getMatterSubsystem().calcBiasForAccelerationConstraints(s, b);
    return -1.0 * b;
}