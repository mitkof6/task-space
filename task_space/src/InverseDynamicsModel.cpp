#include "InverseDynamicsModel.h"

using namespace std;
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
 * This function calculates the total applied forces. In order to calculate them
 * the model must be realized to Stage::Dynamics. This method is typically
 * called in mixed dynamics scheme thus while the model is numerically
 * integrated (forward dynamics) we may be interested in the acting forces so
 * that a controller (e.g. inverse dynamics torque controller) can calculate the
 * controls correctly. If however the original model is realized to
 * Stage::Dynamics the controller's computeControls will be called again and
 * this will cause an infinite loop. To avoid this problem upon first entrance
 * to this function a working copy of the model is created without the
 * controller and this working model is realized to Stage::Dynamics. Each time
 * the working state is updated from the current state of the simulating model
 * and the variables of the working model's working state.
 */
Vector calcTotalForces(const State& s, const Model& model) {
    // create a working instance of the model, upon first entrance to this
    // function
    static Model* workingModel = NULL;
    if (workingModel == NULL) {
        workingModel = model.clone();
        workingModel->updControllerSet().setSize(0);
        workingModel->setUseVisualizer(false);
        workingModel->initSystem();
    }
    // initialize working state from s
    State& workingState = workingModel->updWorkingState();
    workingState.setTime(s.getTime());
    workingModel->initStateWithoutRecreatingSystem(workingState);
    // setting the working state causes segmentation fault on Arch-Linux but 
    // on Windows everything works normal TODO: should investigate
    workingState.setQ(s.getU());
    workingState.setU(s.getU());
    workingState.setZ(s.getZ());
    workingState.setY(s.getY());
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
    // for testing
    //cout << calcCoriolis(s, model) + calcTotalForces(s, model) 
    //    - (calcCoriolis(s, model) + calcGravity(s, model)) << endl;
    // compute all acting forces add Coriolis since they are not accounted
    return  calcCoriolis(s, model) + calcTotalForces(s, model);
    // compute only Coriolis and gravity (works always)
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
