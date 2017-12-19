#include "TaskProjection.h"
#include <simmath/LinearAlgebra.h>

using namespace SimTK;

Matrix calcJpT(const Matrix& NT, const Matrix& JT) {
    return NT * JT;
}

Matrix calcLambdap(const Matrix& J, const Matrix& MInv, const Matrix& JpT) {
    Matrix Lambda;
    auto LambdaInv = J * MInv * JpT;
    FactorSVD svd(LambdaInv);
    svd.inverse(Lambda);
    return Lambda;
}

Matrix calcJBarT(const Matrix& Lambda, const Matrix& J, const Matrix& MInv) {
    return Lambda * J * MInv;
}

Matrix calcNpT(const Matrix& JpT, const Matrix& JBarpT, const Matrix& NT) {
    return (1 - JpT * JBarpT) * NT;
}

Vector calcFt(const Vector& xddot, const Matrix& Lambdap, const Vector& bt,
	      const Matrix& JBarpT, const Vector& taup) {
    return Lambdap * (xddot + bt) + JBarpT * taup;
}

Vector calcTau(const Matrix& JT, const Vector& f) {
    return JT * f;
}
