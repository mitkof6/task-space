#include "TaskPriorityGraph.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "KinematicTask.h"

using namespace std;
using namespace OpenSim;

TaskPriorityGraph::TaskPriorityGraph() {
}

void TaskPriorityGraph::addTask(KinematicTask* task, KinematicTask* parent) {
    // first check whether task exists in the priority list so that we can avoid
    // directed cycles
    for (auto& t : prioritySortedGraph) {
	if (t.first == task) {
	    throw runtime_error("The task already exists in the TaskPriorityGraph");
	}
    }

    // insert task after parent
}
