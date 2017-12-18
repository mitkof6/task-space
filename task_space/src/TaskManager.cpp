#include "TaskManager.h"
#include "TaskPriorityGraph.h"
#include <simmath/LinearAlgebra.h>

using namespace OpenSim;
using namespace SimTK;

TaskManager::TaskManager(TaskPriorityGraph* graph) : taskPriorityGraph(graph) {

}

Vector TaskManager::calcTaskTorques(const State& s) {

    return Vector();
}

Matrix TaskManager::calcLambda(const Matrix& Jt, const Matrix& MInv,
			       const Matrix& NT) {
    Matrix Lambda;
    auto JtT = ~Jt;
    auto LambdaInv = Jt * MInv * NT * JtT;
    FactorSVD svd(LambdaInv);
    svd.inverse(Lambda);
    return Lambda;
}
