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
    taskCache.clear();
    auto graph = taskPriorityGraph->getPrioritySortedGraph();

    // constraint related
    Matrix McInv = constraintModel->McInv(s);
    Matrix NcT = constraintModel->NcT(s);
    Vector bc = constraintModel->bc(s);
    Vector f = calcTotalGeneralizedForces(s, *_model);
    Vector fperp = NcT * f;

    // local variables
    Vector tauTotal(s.getNU(), 0.0); // total task torques
    Matrix NTTotal(s.getNU(), s.getNU()); // total task null space
    Matrix NpT(s.getNU(), s.getNU()); // prioritized null space of higher priority task
    Vector taup(s.getNU(), 0.0); // induced acceleration by higher priority tasks
    NTTotal = 1;
    NpT = NcT;

    for (auto pair : graph) {
	auto task = pair.first;
	auto parent = pair.second;

	if (parent != NULL) {
	    NpT = taskCache[parent].NT;
	    taup = taskCache[parent].tau;
	} else {
	    NpT = 1;
	    taup = 0;
	}

	// calculate task related data
	auto xtddot = task->getGoal(s);
	auto bt = task->b(s);
	auto Jt = task->J(s);
	auto JtT = ~Jt;
	auto JtpT = calcJpT(NpT, JtT);
	auto Lambdatp = calcLambda(Jt, McInv, JtpT);
	auto JBartpT = calcJBarT(Lambdatp, Jt, McInv);
	auto NtT = calcNtT(JtpT, JBartpT);

	// calculate task forces and torques
	auto taue = fperp + bc - taup;
	auto ft = calcFt(Lambdatp, xtddot, bt, JBartpT, taue);
	auto taut = calcTau(JtpT, ft);
	tauTotal += taut;

	// update
	taskCache[task].NT = NtT * NpT;
	taskCache[task].tau = taup + taut;
	NTTotal = NtT * NTTotal;
    }
    // TODO selection matrix
    // compensation forces
    Vector tauResiduals = NTTotal * NcT * (f + bc);
    return tauTotal + tauResiduals;
}
