#include "TaskProjection.h"
#include <simmath/LinearAlgebra.h>

using namespace SimTK;

Matrix calcJpT(const Matrix& J, const Matrix& NT) {
    return NT * ~J;
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

Matrix calcNPT(const Matrix& JpT, const Matrix& JBarpT, const Matrix& NT) {
    return (1 - JpT * JBarpT) * NT;
}

Vector calcft(const Vector& xddot, const Matrix& Lambdap, const Vector& bt,
	      const Matrix& JBarpT, const Vector& taup) {
    return Lambdap * (xddot + bt) + JBarpT * taup;
}
