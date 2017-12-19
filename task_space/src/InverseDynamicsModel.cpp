#include "InverseDynamicsModel.h"

using namespace OpenSim;
using namespace SimTK;

Matrix calcMInv(const State& s, const Model& model) {
    Matrix MInv;
    model.getMatterSubsystem().calcMInv(s, MInv);
    return MInv;
}

Vector calcGravity(const State& s, const Model& model) {
    Vector g;
    model.getMatterSubsystem().multiplyBySystemJacobianTranspose(
	s, model.getGravityForce().getBodyForces(s), g);
    return -g;
}

Vector calcCoriolis(const State& s, const Model& model) {
    Vector c;
    model.getMatterSubsystem().calcResidualForceIgnoringConstraints(
	s, Vector(0), Vector_<SpatialVec>(0), Vector(0), c);
    return c;
}

// void calcTotalGeneralizedForces(const State& s, const OpenSim::Model& model,
//                                 Vector& f) {
//     // update working state
//     State workingState = model.getWorkingState();
//     workingState.updTime() = s.getTime();
//     workingState.updQ() = s.getQ();
//     workingState.updU() = s.getU();
//     workingState.updZ() = s.getZ();
//     workingState.updY() = s.getY();

//     // disable any actuators when computing the total force
//     const OpenSim::Set<OpenSim::Actuator>& as = model.getActuators();
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

Vector calcTotalForces(const State& s, const Model& model) {
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
    Vector bc;
    model.getMatterSubsystem().calcBiasForAccelerationConstraints(s, bc);
    return -bc;
}
