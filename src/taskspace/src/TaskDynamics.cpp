#include "TaskDynamics.h"
#include "TaskPriorityGraph.h"
#include "TaskProjection.h"
#include "InverseDynamicsModel.h"
#include "ConstraintProjection.h"
#include <map>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

TaskDynamics::TaskDynamics(TaskPriorityGraph* graph,
                           ConstraintModel* constraintModel)
    : taskPriorityGraph(graph), constraintModel(constraintModel) {
}

TaskDynamics::TaskDynamicsData TaskDynamics::calcTaskDynamicsData(
    const State& s) {
    TaskDynamicsData data;

    // constraint model
    auto constraintData = constraintModel->calcConstraintData(s);
    auto McInv = constraintData.McInv;
    auto NcT = constraintData.NcT;
    data.bc = constraintData.bc;
    data.f = calcTotalGeneralizedForces(s, *_model);
    auto fperp = NcT * data.f;

    // initialize
    data.tauTasks = Vector(s.getNU(), 0.0);
    data.NgT = Matrix(s.getNU(), s.getNU());
    data.NgT = NcT;

    // current aggregate data
    Matrix NaT(s.getNU(), s.getNU());
    Vector taua(s.getNU(), 0.0);

    // for storing data related to the higher priority tasks
    struct TaskCacheData {
        SimTK::Matrix NaT; // aggregate nullspace
        SimTK::Vector taua; // aggregate generalized forces
    };
    std::map<KinematicTask*, TaskCacheData> taskCache;

    // access the priority graph
    auto graph = taskPriorityGraph->getPrioritySortedGraph();

    // loop through the priority sorted graph [high, low]
    for (auto pair : graph) {
        auto task = pair.first; // current task
        auto parent = pair.second; // higher priority task

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
        auto tau = fperp + data.bc - taua;
        auto ft = calcFt(Lambdatp, xtddot, bt, JBartpT, tau);
        auto taut = calcTau(JtpT, ft);

        // cache the aggregate information of the task
        taskCache[task].NaT = NtT * NaT;
        taskCache[task].taua = taua + taut;

        // update total task forces and nullspace
        data.tauTasks += taut;
        data.NgT = NtT * data.NgT;
    }
    return data;
}