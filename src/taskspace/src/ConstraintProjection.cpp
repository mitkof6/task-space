#include "ConstraintProjection.h"
#include "InverseDynamicsModel.h"

using namespace OpenSim;
using namespace SimTK;

Matrix calcLambda(const Matrix& Phi, const Matrix& MInv) {
    auto LambdaInv = Phi * MInv * ~Phi;
    Matrix Lambda;
    FactorSVD svd(LambdaInv);
    svd.inverse(Lambda);
    return Lambda;
}

/******************************************************************************/

UnconstraintModel::ConstraintData  UnconstraintModel::calcConstraintData(
    const SimTK::State& s) const {
    ConstraintData data;
    data.McInv = calcMInv(s, *_model);
    data.bc = Vector(s.getNU(), 0.0);
    data.NcT = Matrix(s.getNU(), s.getNU());
    data.NcT = 1;
    return data;
}

/******************************************************************************/

DeSapioModel::ConstraintData  DeSapioModel::calcConstraintData(
    const SimTK::State& s) const {
    ConstraintData data;
    // McInv
    data.McInv = calcMInv(s, *_model);
    // bc
    auto Phi = calcConstraintJacobian(s, *_model);
    auto PhiT = ~Phi;
    auto Lambdac = calcLambda(Phi, data.McInv);
    auto b = calcConstraintBias(s, *_model);
    data.bc = -1.0 * PhiT * Lambdac * b;
    // NcT
    auto PhiBarT = Lambdac * Phi * data.McInv;
    data.NcT = 1 - PhiT * PhiBarT;
    return data;
}

/******************************************************************************/

AghiliModel::ConstraintData  AghiliModel::calcConstraintData(
    const SimTK::State& s) const {
    ConstraintData data;
    // NcT
    auto Phi = calcConstraintJacobian(s, *_model);
    Matrix PhiInv;
    FactorSVD PhiSVD(Phi);
    PhiSVD.inverse(PhiInv);
    data.NcT = 1 - PhiInv * Phi; // Nc^T = Nc due to MPP properties
    // McInv
    auto M = calcM(s, *_model);
    auto Ms = M + data.NcT * M - ~(data.NcT * M);
    FactorSVD MsSVD(Ms);
    MsSVD.inverse(data.McInv);
    // bc
    auto b = calcConstraintBias(s, *_model);
    data.bc = -1.0 * M * PhiInv * b;
    return data;
}