#include "TaskManager.h"
#include "TaskPriorityGraph.h"

using namespace OpenSim;
using namespace SimTK;

TaskManager::TaskManager(TaskPriorityGraph* graph) : taskPriorityGraph(graph) {

}

Vector TaskManager::calcTaskTorques(const State& s) {


    return Vector();
}
