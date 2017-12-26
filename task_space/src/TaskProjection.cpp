#include "TaskProjection.h"
#include <simmath/LinearAlgebra.h>

using namespace SimTK;

Matrix calcJpT(const Matrix& NaT, const Matrix& JT) {
    return NaT * JT;
}

Matrix calcLambda(const Matrix& J, const Matrix& MInv, const Matrix& JT) {
    Matrix Lambda;
    auto LambdaInv = J * MInv * JT;
    FactorSVD svd(LambdaInv);
    svd.inverse(Lambda);
    return Lambda;
}

Matrix calcJBarT(const Matrix& Lambda, const Matrix& J, const Matrix& MInv) {
    return Lambda * J * MInv;
}

Matrix calcNtT(const Matrix& JT, const Matrix& JBarT) {
    return 1 - JT * JBarT;
}

Matrix calcNpT(const Matrix& NtT, const Matrix& NaT) {
    return NtT * NaT;
}

Vector calcFt(const Matrix& Lambda, const Vector& xddot, const Vector& bt,
	      const Matrix& JBarT, const Vector& tau) {
    return Lambda * (xddot + bt) + JBarT * tau;
}

Vector calcTau(const Matrix& JT, const Vector& f) {
    return JT * f;
}
