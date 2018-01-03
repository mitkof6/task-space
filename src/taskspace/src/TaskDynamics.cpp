#include "TaskDynamics.h"
#include "TaskProjection.h"
#include "InverseDynamicsModel.h"
#include "ConstraintProjection.h"
#include <map>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

TaskDynamics::TaskDynamics(ConstraintModel* constraintModel)
    : constraintModel(constraintModel) {
}

void TaskDynamics::addTask(KinematicTask* task, KinematicTask* parent) {
    // first check whether task exists in the priority list so that we can
    // avoid directed cycles
    for (auto& t : prioritySortedGraph) {
        if (t.first == task) {
            throw TaskExistsInGraphException(
                "The task already exists in the priority graph (avoid cyclic graph)");
        }
    }
    // find the parent task and insert after
    auto parentIt = find_if(prioritySortedGraph.begin(),
                            prioritySortedGraph.end(),
                            [&](const pair<KinematicTask*, KinematicTask*>& a) {
        return a.first == parent;
    });
    if (parent != NULL && parentIt == prioritySortedGraph.end()) {
        throw ParentNotInGraphException(
            "The parent does not exist in the priority graph");
    }
    // insert after
    if (parentIt != prioritySortedGraph.end()) {
        parentIt++;
    }
    prioritySortedGraph.insert(parentIt, make_pair(task, parent));
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

    // loop through the priority sorted graph [high, low]
    for (auto pair : prioritySortedGraph) {
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