#include "TaskManager.h"
#include "TaskPriorityGraph.h"
#include "TaskProjection.h"
#include "InverseDynamicsModel.h"
#include "ConstraintProjection.h"

using namespace OpenSim;
using namespace SimTK;

TaskManager::TaskManager(TaskPriorityGraph* graph,
			 ConstraintModel* constraintModel)
    : taskPriorityGraph(graph), constraintModel(constraintModel) {

}

Vector TaskManager::calcTaskTorques(const State& s) {
    auto graph = taskPriorityGraph->getPrioritySortedGraph();

    Matrix McInv = constraintModel->McInv(s);
    Matrix NcT = constraintModel->NcT(s);
    Matrix bc = constraintModel->bc(s);
    Vector f = calcTotalGeneralizedForces(s, *_model);
    Vector fPerp = NcT * f;

    Matrix NpT = NcT;
    Vector tauP(s.getNU(), 0.0);

    for (auto pair : graph) {
	auto task = pair.first;
	auto parent = pair.second;

    }

    return Vector();
}
