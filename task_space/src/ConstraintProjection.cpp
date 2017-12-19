#include "ConstraintProjection.h"
#include "InverseDynamicsModel.h"

using namespace OpenSim;
using namespace SimTK;

Matrix calcLambda(const Matrix& Phi, const Matrix& MInv) {
    Matrix Lambda;
    auto LambdaInv = Phi * MInv * ~Phi;
    FactorSVD svd(LambdaInv);
    svd.inverse(Lambda);
    return Lambda;
}

/******************************************************************************/

Matrix UnconstraintModel::McInv(const State& s) const {
    return calcMInv(s, *_model);
}

Vector UnconstraintModel::bc(const State& s) const {
    return Vector(s.getNU(), 0.0);
}

Matrix UnconstraintModel::NcT(const State& s) const {
    Matrix eye(s.getNU(), s.getNU());
    eye = 1;
    return eye;
}

/******************************************************************************/

Matrix DeSapioModel::McInv(const State& s) const {
    return calcMInv(s, *_model);
}

Vector DeSapioModel::bc(const State& s) const {
    return  -1.0 * ~Phi(s) * Lambdac(s) * calcConstraintBias(s, *_model);
}

Matrix DeSapioModel::NcT(const State& s) const {
    return 1 - ~Phi(s) * PhiBarT(s);
}

Matrix DeSapioModel::Lambdac(const State& s) const {
    return calcLambda(Phi(s), McInv(s));
}

Matrix DeSapioModel::Phi(const State& s) const {
    return calcConstraintJacobian(s, *_model);
}

Matrix DeSapioModel::PhiBarT(const State& s) const {
    return Lambdac(s) * Phi(s) * McInv(s);
}

/******************************************************************************/
