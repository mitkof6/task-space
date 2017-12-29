#include "TaskPriorityGraph.h"

using namespace std;
using namespace OpenSim;

void TaskPriorityGraph::addTask(KinematicTask* task, KinematicTask* parent) {
    // first check whether task exists in the priority list so that we can avoid
    // directed cycles
    for (auto& t : prioritySortedGraph) {
        if (t.first == task) {
            throw TaskExistsInGraphException(
                "The task already exists in the TaskPriorityGraph (avoid cyclic graph)");
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
    prioritySortedGraph.insert(++parentIt, make_pair(task, parent));
}

const ListChildParent& TaskPriorityGraph::getPrioritySortedGraph() const {
    return prioritySortedGraph;
}
