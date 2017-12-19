#include "TaskManager.h"
#include "TaskPriorityGraph.h"
#include "TaskProjection.h"

using namespace OpenSim;
using namespace SimTK;

TaskManager::TaskManager(std::shared_ptr<TaskPriorityGraph> graph)
    : taskPriorityGraph(graph) {

}

Vector TaskManager::calcTaskTorques(const State& s) {


    return Vector();
}
