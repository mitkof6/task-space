#include "TaskManager.h"
#include "TaskPriorityGraph.h"
#include "TaskProjection.h"
#include "InverseDynamicsModel.h"
#include "ConstraintProjection.h"

using namespace std;
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
    // loop through the priority sorted graph [high, low]
    for (auto pair : graph) {
	auto task = pair.first;
	auto parent = pair.second;
	// get parent's null space and force contribution
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
    // nullspace forces
    Vector tauNullspace = NTTotal * NcT * (f + bc);
    appendAnalytics(s, tauTotal, tauNullspace);
    return tauTotal + tauNullspace;
}

void TaskManager::printResults(std::string prefix, std::string dir) {
    analytics.print(dir + "/" + prefix + "_TaskManager.sto");
}

void TaskManager::extendInitStateFromProperties(State& s) const {
    Super::extendInitStateFromProperties(s);
    // construct labels
    Array<string> labels;
    labels.append("time");
    const auto& cs = _model->getCoordinateSet();
    for (int i = 0; i < cs.getSize(); i++) {
	labels.append("task_" + cs[i].getName());
    }
    for (int i = 0; i < cs.getSize(); i++) {
	labels.append("nullspace_" + cs[i].getName());
    }
    labels.append("task_magnitude");
    labels.append("nullspace_magnitude");
    labels.append("total_magnitude");
    analytics.setColumnLabels(labels);
    analytics.reset(0);
}

void TaskManager::appendAnalytics(const SimTK::State& s,
				  const SimTK::Vector& taskForces,
				  const SimTK::Vector& nullspaceForces) {
    int n = _model->getNumCoordinates();
    Vector data(2 * n + 3, 0.0);
    data(0, n) = taskForces;
    data(n, n) = nullspaceForces;
    data[data.size() - 3] = taskForces.norm();
    data[data.size() - 2] = nullspaceForces.norm();
    data[data.size() - 1] = data[data.size() - 3] + data[data.size() - 2];
    analytics.append(s.getTime(), data.size(), &data[0], true);
}
