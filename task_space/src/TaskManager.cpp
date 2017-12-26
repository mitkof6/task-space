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
    // constraint model
    auto constraintData = constraintModel->calcConstraintData(s);
    auto McInv = constraintData.McInv;
    auto NcT = constraintData.NcT;
    auto bc = constraintData.bc;
    auto f = calcTotalGeneralizedForces(s, *_model);
    auto fperp = NcT * f;
    // local variables
    Vector tauTasks(s.getNU(), 0.0); // the total task torques
    Matrix NgT(s.getNU(), s.getNU()); // the total nullspace
    NgT = NcT;
    Matrix NaT(s.getNU(), s.getNU()); // the aggregate nullspace of the higher
				      // priority tasks
    Vector taua(s.getNU(), 0.0); // the aggregate induced task torques of the
				 // higher priority tasks

    // loop through the priority sorted graph [high, low]
    for (auto pair : graph) {
	auto task = pair.first; // current task
	auto parent = pair.second; // higher priority
	// get parent's nullspace and force contribution
	if (parent != NULL) {
	    NaT = taskCache[parent].NaT;
	    taua = taskCache[parent].taua;
	} else {
	    NaT = NcT; // constraints have the highest priority
	    taua = 0;
	}
	// calculate task related data
	auto xtddot = task->getGoal(s);
	auto bt = task->b(s);
	auto Jt = task->J(s);
	auto JtT = ~Jt;
	auto JtpT = calcJpT(NaT, JtT);
	auto Lambdatp = calcLambda(Jt, McInv, JtpT);
	auto JBartpT = calcJBarT(Lambdatp, Jt, McInv);
	auto NtT = calcNtT(JtpT, JBartpT);
	// calculate task forces and torques
	auto tau = fperp + bc - taua;
	auto ft = calcFt(Lambdatp, xtddot, bt, JBartpT, tau);
	auto taut = calcTau(JtpT, ft);
	// cache the aggregate information of the task
	taskCache[task].NaT = NtT * NaT;
	taskCache[task].taua = taua + taut;
	// update total task forces and nullspace
	tauTasks += taut;
	NgT = NtT * NgT;
    }
    // TODO selection matrix and constraint forces analytics
    auto tauNullspace = NgT * (f + bc);
    appendAnalytics(s, tauTasks, tauNullspace);
    return tauTasks + tauNullspace;
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
    // setup storage
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
