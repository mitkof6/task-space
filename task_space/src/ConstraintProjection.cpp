#include "ConstraintProjection.h"
#include "InverseDynamicsModel.h"

using namespace OpenSim;
using namespace SimTK;

Matrix UnconstraintModel::McInv(const State& s) const {

    return Matrix();
}

SimTK::Vector bc(const SimTK::State& s) const {
    return Vector();
}
