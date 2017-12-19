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

Matrix calcMInv(const State& s, const Model& model) {
    Matrix MInv;
    model.getMatterSubsystem().calcMInv(s, MInv);
    return MInv;
}

// void calcTotalGeneralizedForces(const State& s, const Model& model,
//                                 Vector& f) {
//     // update working state
//     State workingState = model.getWorkingState();
//     workingState.updTime() = s.getTime();
//     workingState.updQ() = s.getQ();
//     workingState.updU() = s.getU();
//     workingState.updZ() = s.getZ();
//     workingState.updY() = s.getY();

//     // disable any actuators when computing the total force
//     const Set<Actuator>& as = model.getActuators();
//     for (int i = 0; i < as.getSize(); i++) {
//         as[i].setDisabled(workingState, true);
//     }

//     // generalized forces and torques should be accessed at a Dynamics stage
//     model.realizeDynamics(workingState);

//     const Vector_<SpatialVec>& forces = model.getMultibodySystem()
//         .getRigidBodyForces(workingState, Stage::Dynamics);
//     const Vector& torques = model.getMultibodySystem().getMobilityForces(
//         workingState, Stage::Dynamics);

//     model.getMatterSubsystem().multiplyBySystemJacobianTranspose(workingState,
//                                                                  forces, f);

//     // in our conviction we subtract their contribution
//     f = -1.0 * torques - f;
// }

Vector calcTotalGeneralizedForces(const State& s, const Model& model) {
    // TODO calcTotalGeneralizedForces(s, workingModel, value); value =
    // calcCoriolis(s) + value;// add Coriolis because they are not computed
    return calcCoriolis(s, model) + calcGravity(s, model);
}

Matrix calcConstraintJacobian(const State& s, Model& model) {
    Matrix Phi;
    model.getMatterSubsystem().calcG(s, Phi);
    return Phi;
}

Vector calcConstraintBias(const State& s, Model& model) {
    Vector b;
    model.getMatterSubsystem().calcBiasForAccelerationConstraints(s, b);
    return b;
}
